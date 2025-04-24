#include "hmcstlock.h"

#include <assert.h>
#include <pthread.h>
#include <sched.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>

/*────────── Constants for timeouts ─────────────────────────────────*/
#define BIG_TIMEOUT     3000000000UL   /* 1 s     */
#define MID_TIMEOUT      300000000UL   /* 100 ms  */
#define SMALL_TIMEOUT    30000000UL    /* 10 ms   */
#define ZERO_TIMEOUT          0UL      /* 0 ns    */

#define NSLEEP(ms)       do { struct timespec ts = {0, (ms)*1000000}; \
                              nanosleep(&ts, NULL); } while (0)

/*────────── Hierarchical lock factory ─────────────────────────────*/
static hmcslock_t *mk_lock(size_t levels, uint32_t thr)
{
    hmcslock_t *a = calloc(levels, sizeof *a);
    if (!a) { perror("calloc"); exit(EXIT_FAILURE); }
    for (size_t i = 0; i < levels; ++i) {
        a[i].parent    = (i + 1 < levels) ? &a[i + 1] : NULL;
        a[i].threshold = thr;
        vatomicptr_write_rlx(&a[i].lock, NULL);
        hmcst_qnode_init(&a[i].qnode);
    }
    return a;
}

/*────────────────────── Unit Tests ─────────────────────────────────*/
static void t_leaf_ok(void)
{
    hmcslock_t *L = mk_lock(1, 1);
    hmcs_node_t q; hmcst_qnode_init(&q);
    assert(hmcst_acquire(L, &q, 1, BIG_TIMEOUT) == HMCST_SUCCESS);
    hmcst_release(L, &q, 1);
    free(L);
    puts("leaf_success ✓");
}

static void t_leaf_timeout_reuse(void)
{
    hmcslock_t *L = mk_lock(1, 1);
    hmcs_node_t q1, q2;
    hmcst_qnode_init(&q1);
    hmcst_qnode_init(&q2);

    assert(hmcst_acquire(L, &q1, 1, BIG_TIMEOUT) == HMCST_SUCCESS);
    assert(hmcst_acquire(L, &q2, 1, SMALL_TIMEOUT) == HMCST_TIMEOUT);
    hmcst_release(L, &q1, 1);

    assert(hmcst_acquire(L, &q2, 1, BIG_TIMEOUT) == HMCST_SUCCESS);
    hmcst_release(L, &q2, 1);

    free(L);
    puts("leaf_timeout_reacquire ✓");
}

static void t_inherited_C_and_P(void)
{
    hmcslock_t *L = mk_lock(2, 2);
    hmcs_node_t q; hmcst_qnode_init(&q);

    /* Inherit as cohort leader */
    vatomic64_write_rlx(&q.status, HMCLOCK_COHORT_START);
    assert(hmcst_acquire(&L[0], &q, 2, BIG_TIMEOUT) == HMCST_SUCCESS);
    hmcst_release(&L[0], &q, 2);

    /* Inherit from parent */
    vatomic64_write_rlx(&q.status, HMCLOCK_ACQUIRE_PARENT);
    assert(hmcst_acquire(&L[0], &q, 2, BIG_TIMEOUT) == HMCST_SUCCESS);
    hmcst_release(&L[0], &q, 2);

    free(L);
    puts("inherited_C_P ✓");
}

static void t_three_level_middle_abort(void)
{
    hmcslock_t *L = mk_lock(3, 1);
    hmcs_node_t mid;  hmcst_qnode_init(&mid);
    assert(hmcst_acquire(&L[1], &mid, 1, BIG_TIMEOUT) == HMCST_SUCCESS);

    hmcs_node_t leaf; hmcst_qnode_init(&leaf);
    assert(hmcst_acquire(&L[0], &leaf, 3, SMALL_TIMEOUT) == HMCST_TIMEOUT);

    hmcst_release(&L[1], &mid, 1);
    free(L);
    puts("middle_abort_three_levels ✓");
}

/*────────────────── Multi-Thread Tests ─────────────────────────────*/
static hmcslock_t *lock1;
static void *worker_mutex(void *arg)
{
    (void)arg;
    for (int i = 0; i < 200000; i++) {
        hmcs_node_t q; hmcst_qnode_init(&q);
        assert(hmcst_acquire(lock1, &q, 1, BIG_TIMEOUT) == HMCST_SUCCESS);
        hmcst_release(lock1, &q, 1);
    }
    return NULL;
}
static void t_mutex_enforced(void)
{
    pthread_t threads[8];
    lock1 = mk_lock(1, 1);
    for (int i = 0; i < 8; i++)
        pthread_create(&threads[i], NULL, worker_mutex, NULL);
    for (int i = 0; i < 8; i++)
        pthread_join(threads[i], NULL);
    free(lock1);
    puts("mutex_enforcement ✓");
}

static hmcslock_t *lock2;
static atomic_int pass_cnt;
static void *worker_pass(void *arg)
{
    (void)arg;
    hmcs_node_t q; hmcst_qnode_init(&q);
    assert(hmcst_acquire(lock2, &q, 2, BIG_TIMEOUT) == HMCST_SUCCESS);
    atomic_fetch_add(&pass_cnt, 1);
    hmcst_release(lock2, &q, 2);
    return NULL;
}
static void t_local_pass_threshold(void)
{
    pthread_t a, b, c;
    lock2 = mk_lock(2, 2);
    atomic_store(&pass_cnt, 0);

    pthread_create(&a, NULL, worker_pass, NULL);
    pthread_create(&b, NULL, worker_pass, NULL);
    NSLEEP(1);
    pthread_create(&c, NULL, worker_pass, NULL);

    pthread_join(a, NULL);
    pthread_join(b, NULL);
    pthread_join(c, NULL);

    assert(atomic_load(&pass_cnt) == 3);
    free(lock2);
    puts("local_pass_threshold ✓");
}

static hmcslock_t *imp_lock;
static atomic_bool ready, gone;
static void cleanup_release(void *arg)
{
    hmcs_node_t *q = arg;
    hmcst_release(imp_lock, q, 1);
    atomic_store(&gone, true);
}
static void *blk(void *arg)
{
    hmcs_node_t q; hmcst_qnode_init(&q);
    assert(hmcst_acquire(imp_lock, &q, 1, BIG_TIMEOUT) == HMCST_SUCCESS);
    pthread_cleanup_push(cleanup_release, &q);
    atomic_store(&ready, true);
    for (;;) { pthread_testcancel(); NSLEEP(10); }
    pthread_cleanup_pop(0);
    return NULL;
}
static void *imp(void *arg)
{
    pthread_t tid = *(pthread_t *)arg;
    hmcs_node_t q; hmcst_qnode_init(&q);

    assert(hmcst_acquire(imp_lock, &q, 1, SMALL_TIMEOUT) == HMCST_TIMEOUT);
    pthread_cancel(tid);
    while (!atomic_load(&gone)) sched_yield();
    assert(hmcst_acquire(imp_lock, &q, 1, BIG_TIMEOUT) == HMCST_SUCCESS);
    hmcst_release(imp_lock, &q, 1);
    return NULL;
}
static void t_impatient(void)
{
    pthread_t tblk, timp;
    imp_lock = mk_lock(1, 1);
    atomic_store(&ready, false);
    atomic_store(&gone,  false);

    pthread_create(&tblk, NULL, blk, NULL);
    while (!atomic_load(&ready)) sched_yield();
    pthread_create(&timp, NULL, imp, &tblk);

    pthread_join(timp, NULL);
    pthread_join(tblk, NULL);
    free(imp_lock);
    puts("impatient_handshake ✓");
}

/*────────────── Limits & Stress Tests ──────────────────────────────*/
static void t_zero_timeout(void)
{
    hmcslock_t *L = mk_lock(1, 1);
    hmcs_node_t q1, q2;
    hmcst_qnode_init(&q1);
    hmcst_qnode_init(&q2);

    assert(hmcst_acquire(L, &q1, 1, ZERO_TIMEOUT) == HMCST_SUCCESS);
    assert(hmcst_acquire(L, &q2, 1, ZERO_TIMEOUT) == HMCST_TIMEOUT);
    hmcst_release(L, &q1, 1);

    assert(hmcst_acquire(L, &q2, 1, BIG_TIMEOUT) == HMCST_SUCCESS);
    hmcst_release(L, &q2, 1);
    free(L);

    puts("t_zero_timeout ✓");
}

static void t_zero_vs_contention(void)
{
    hmcslock_t *L = mk_lock(1, 1);
    hmcs_node_t q1, q2;
    hmcst_qnode_init(&q1);
    hmcst_qnode_init(&q2);

    assert(hmcst_acquire(L, &q1, 1, ZERO_TIMEOUT) == HMCST_SUCCESS);
    hmcst_release(L, &q1, 1);
    assert(hmcst_acquire(L, &q1, 1, BIG_TIMEOUT) == HMCST_SUCCESS);
    assert(hmcst_acquire(L, &q2, 1, ZERO_TIMEOUT) == HMCST_TIMEOUT);
    hmcst_release(L, &q1, 1);

    for (int i = 0; i < 100; i++) {
        hmcst_qnode_init(&q1);
        assert(hmcst_acquire(L, &q1, 1, BIG_TIMEOUT) == HMCST_SUCCESS);
        hmcst_release(L, &q1, 1);

        hmcst_qnode_init(&q1);
        assert(hmcst_acquire(L, &q1, 1, ZERO_TIMEOUT) == HMCST_SUCCESS);
        hmcst_release(L, &q1, 1);

        hmcst_qnode_init(&q1);
        hmcst_qnode_init(&q2);
        assert(hmcst_acquire(L, &q1, 1, BIG_TIMEOUT) == HMCST_SUCCESS);
        assert(hmcst_acquire(L, &q2, 1, ZERO_TIMEOUT) == HMCST_TIMEOUT);
        hmcst_release(L, &q1, 1);
    }

    free(L);
    puts("t_zero_vs_contention ✓");
}

static void t_two_level_escalation(void)
{
    hmcslock_t *L = mk_lock(2, 1);
    hmcs_node_t q; hmcst_qnode_init(&q);
    for (int i = 0; i < 10; i++) {
        assert(hmcst_acquire(&L[0], &q, 2, BIG_TIMEOUT) == HMCST_SUCCESS);
        hmcst_release(&L[0], &q, 2);
    }
    free(L);
    puts("t_two_level_escalation ✓");
}

static void *thr3(void *arg)
{
    hmcslock_t *L = arg;
    for (int i = 0; i < 1000; i++) {
        hmcs_node_t q; hmcst_qnode_init(&q);
        assert(hmcst_acquire(L, &q, 3, BIG_TIMEOUT) == HMCST_SUCCESS);
        hmcst_release(L, &q, 3);
    }
    return NULL;
}
static void t_three_threads_three_levels(void)
{
    hmcslock_t *L = mk_lock(3, 4);
    pthread_t t[3];
    for (int i = 0; i < 3; i++) pthread_create(&t[i], NULL, thr3, L);
    for (int i = 0; i < 3; i++) pthread_join(t[i], NULL);
    free(L);
    puts("t_three_threads_three_levels ✓");
}

static void t_many_reuses(void)
{
    hmcslock_t *L = mk_lock(2, 2);
    hmcs_node_t q; hmcst_qnode_init(&q);
    for (int i = 0; i < 100000; i++) {
        assert(hmcst_acquire(L, &q, 1, BIG_TIMEOUT) == HMCST_SUCCESS);
        hmcst_release(L, &q, 1);
    }
    free(L);
    puts("t_many_reuses ✓");
}

/*───────────────── Additional HMCS-T Tests ─────────────────────────*/
static void t_multiple_abandon_levels(void)
{
    hmcslock_t *L = mk_lock(2, 1);
    hmcs_node_t q1, q2, q3, q4;
    hmcst_qnode_init(&q1);
    hmcst_qnode_init(&q2);
    hmcst_qnode_init(&q3);
    hmcst_qnode_init(&q4);

    assert(hmcst_acquire(L, &q1, 1, BIG_TIMEOUT) == HMCST_SUCCESS);
    assert(hmcst_acquire(L, &q2, 1, SMALL_TIMEOUT) == HMCST_TIMEOUT);
    assert(hmcst_acquire(L, &q3, 1, SMALL_TIMEOUT) == HMCST_TIMEOUT);
    assert(hmcst_acquire(L, &q4, 1, SMALL_TIMEOUT) == HMCST_TIMEOUT);

    hmcst_release(L, &q1, 1);

    hmcs_node_t q5; hmcst_qnode_init(&q5);
    assert(hmcst_acquire(L, &q5, 1, BIG_TIMEOUT) == HMCST_SUCCESS);
    hmcst_release(L, &q5, 1);

    /* Level 2 sequence */
    hmcs_node_t r1, r2, r3, r4;
    hmcst_qnode_init(&r1);
    hmcst_qnode_init(&r2);
    hmcst_qnode_init(&r3);
    hmcst_qnode_init(&r4);

    assert(hmcst_acquire(L, &r1, 2, BIG_TIMEOUT) == HMCST_SUCCESS);
    assert(hmcst_acquire(L, &r2, 2, SMALL_TIMEOUT) == HMCST_TIMEOUT);
    assert(hmcst_acquire(L, &r3, 2, SMALL_TIMEOUT) == HMCST_TIMEOUT);
    assert(hmcst_acquire(L, &r4, 2, SMALL_TIMEOUT) == HMCST_TIMEOUT);

    hmcst_release(L, &r1, 2);

    hmcs_node_t r5; hmcst_qnode_init(&r5);
    assert(hmcst_acquire(L, &r5, 2, BIG_TIMEOUT) == HMCST_SUCCESS);
    hmcst_release(L, &r5, 2);

    free(L);
    puts("t_multiple_abandon_levels ✓");
}

static void t_multiple_abandon(void)
{
    hmcslock_t *L = mk_lock(1, 1);
    hmcs_node_t q1, q2, q3, q4;
    hmcst_qnode_init(&q1);
    hmcst_qnode_init(&q2);
    hmcst_qnode_init(&q3);
    hmcst_qnode_init(&q4);

    assert(hmcst_acquire(L, &q1, 1, BIG_TIMEOUT) == HMCST_SUCCESS);
    assert(hmcst_acquire(L, &q2, 1, SMALL_TIMEOUT) == HMCST_TIMEOUT);
    assert(hmcst_acquire(L, &q3, 1, SMALL_TIMEOUT) == HMCST_TIMEOUT);
    assert(hmcst_acquire(L, &q4, 1, SMALL_TIMEOUT) == HMCST_TIMEOUT);

    hmcst_release(L, &q1, 1);

    hmcs_node_t q5; hmcst_qnode_init(&q5);
    assert(hmcst_acquire(L, &q5, 1, BIG_TIMEOUT) == HMCST_SUCCESS);
    hmcst_release(L, &q5, 1);

    free(L);
    puts("t_multiple_abandon ✓");
}

/*───────────────── Abandon Edge-Case Tests ─────────────────────────*/
static inline uint64_t status_of(hmcs_node_t *qn) {
    return vatomic64_read_acq(&qn->status);
}

static void t_abandon_edge_cases(void)
{
    puts("=== t_abandon_edge_cases ===");

    /* Case 1: single level, no successor */
    {
        hmcslock_t *L = mk_lock(1, 1);
        hmcs_node_t a, b;
        hmcst_qnode_init(&a);
        hmcst_qnode_init(&b);

        assert(hmcst_acquire(L, &a, 1, BIG_TIMEOUT) == HMCST_SUCCESS);
        assert(hmcst_acquire(L, &b, 1, SMALL_TIMEOUT) == HMCST_TIMEOUT);
        assert(status_of(&b) == HMCST_STATUS_A);

        hmcst_release(L, &a, 1);
        assert(hmcst_acquire(L, &b, 1, BIG_TIMEOUT) == HMCST_SUCCESS);
        hmcst_release(L, &b, 1);

        free(L);
        puts(" case1_single_level_no_succ ✓");
    }

    /* Case 2: two levels, leaf abandon */
    {
        hmcslock_t *L = mk_lock(2, 1);
        hmcs_node_t a, b;
        hmcst_qnode_init(&a);
        hmcst_qnode_init(&b);

        assert(hmcst_acquire(L, &a, 2, BIG_TIMEOUT) == HMCST_SUCCESS);
        assert(hmcst_acquire(L, &b, 2, SMALL_TIMEOUT) == HMCST_TIMEOUT);
        assert(status_of(&b) == HMCST_STATUS_A);

        hmcst_release(L, &a, 2);
        assert(hmcst_acquire(L, &b, 2, BIG_TIMEOUT) == HMCST_SUCCESS);
        hmcst_release(L, &b, 2);

        free(L);
        puts(" case2_two_level_leaf_abandon ✓");
    }

    /* Case 3: U-case escalation */
    {
        hmcslock_t *L = mk_lock(2, 1);
        hmcs_node_t n; hmcst_qnode_init(&n);
        vatomic64_write_rel(&n.status, HMCST_STATUS_U);

        hmcst_abandon(L, &n, 2);
        assert(status_of(&n) == HMCLOCK_COHORT_START);
        assert(status_of(&L[1].qnode) == HMCLOCK_COHORT_START);

        free(L);
        puts(" case3_U_case_escalate ✓");
    }

    /* Case 4: chained abandons at single level */
    {
        hmcslock_t *L = mk_lock(1, 1);
        hmcs_node_t q1, q2, q3, q4;
        hmcst_qnode_init(&q1);
        hmcst_qnode_init(&q2);
        hmcst_qnode_init(&q3);
        hmcst_qnode_init(&q4);

        assert(hmcst_acquire(L, &q1, 1, BIG_TIMEOUT) == HMCST_SUCCESS);
        assert(hmcst_acquire(L, &q2, 1, SMALL_TIMEOUT) == HMCST_TIMEOUT);
        assert(hmcst_acquire(L, &q3, 1, SMALL_TIMEOUT) == HMCST_TIMEOUT);
        assert(hmcst_acquire(L, &q4, 1, SMALL_TIMEOUT) == HMCST_TIMEOUT);

        hmcst_release(L, &q1, 1);

        hmcs_node_t q5; hmcst_qnode_init(&q5);
        assert(hmcst_acquire(L, &q5, 1, BIG_TIMEOUT) == HMCST_SUCCESS);
        hmcst_release(L, &q5, 1);

        free(L);
        puts(" case4_chain_abandon_levels ✓");
    }

    puts("=== t_abandon_edge_cases OK ===\n");
}

/*────────────────── Combined Edge-Case Test ─────────────────────────*/
static void t_edge_cases_acquire_abandon_release(void)
{
    hmcslock_t *L = mk_lock(2, 1);
    hmcs_node_t q1, q2, q3;
    hmcst_qnode_init(&q1);
    hmcst_qnode_init(&q2);
    hmcst_qnode_init(&q3);

    /* Try-lock success/fail */
    assert(hmcst_acquire(L, &q1, 1, 0) == HMCST_SUCCESS);
    assert(hmcst_acquire(L, &q2, 1, 0) == HMCST_TIMEOUT);
    hmcst_release(L, &q1, 1);

    /* Reuse after abandon */
    vatomic64_write_rel(&q1.status, HMCST_STATUS_A);
    assert(hmcst_acquire(L, &q1, 1, BIG_TIMEOUT) == HMCST_SUCCESS);
    hmcst_release(L, &q1, 1);

    /* Explicit abandon when already unlocked */
    vatomic64_write_rel(&q1.status, HMCST_STATUS_U);
    hmcst_abandon(L, &q1, 2);
    assert(vatomic64_read_rlx(&q1.status) == HMCLOCK_COHORT_START);
    assert(vatomic64_read_rlx(&L[1].qnode.status) == HMCLOCK_COHORT_START);

    /* Release escalation */
    assert(hmcst_acquire(L, &q3, 2, BIG_TIMEOUT) == HMCST_SUCCESS);
    hmcst_release(L, &q3, 2);
    assert(vatomic64_read_rlx(&q3.status) == HMCST_STATUS_R);
    assert(REAL_QNODE(vatomicptr_read_rlx(&L[1].lock)) == NULL);

    free(L);
    puts("t_edge_cases_acquire_abandon_release ✓");
}

/*────────────────────────── Main ───────────────────────────────────*/
int main(void)
{
    /* Unit tests */
    t_leaf_ok();
    t_leaf_timeout_reuse();
    t_inherited_C_and_P();
    t_three_level_middle_abort();

    /* Multi-threaded tests */
    t_mutex_enforced();
    t_local_pass_threshold();
    t_impatient();

    /* Limits & stress tests */
    t_zero_timeout();
    t_zero_vs_contention();
    t_two_level_escalation();
    t_three_threads_three_levels();
    t_many_reuses();

    /* Additional HMCS-T tests */
    t_multiple_abandon_levels();
    t_multiple_abandon();

    /* Abandon edge-case tests */
    t_abandon_edge_cases();

    /* Combined acquire/abandon/release */
    t_edge_cases_acquire_abandon_release();

    puts("✅ Tous les scénarios HMCS-T validés");
    return 0;
}
