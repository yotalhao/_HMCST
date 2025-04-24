#ifndef HMCSTLOCK_H
#define HMCSTLOCK_H

/*──────────────── Dependencies and Base Definitions ───────────────────*/
// Core dependencies for HMCS-T lock implementation
#include "hmcslock.h"
#include <assert.h>
#include <stdatomic.h>
#include <time.h>
#include <stdint.h>
#include <sched.h>
#include <stdio.h>

// Yield CPU to reduce contention
#define cpu_relax() sched_yield()

/*──────────────── Cohort and Parent Constants ────────────────────────*/
// Constants for cohort management and parent acquisition
#define HMCLOCK_COHORT_START   0x5UL          // Marks a cohort leader
#define HMCLOCK_MAX            UINT64_MAX     // Maximum value for counters
#define HMCLOCK_ACQUIRE_PARENT (HMCLOCK_MAX - 1) // Indicates parent-passed lock
#define HMCLOCK_WAIT           HMCLOCK_MAX    // Waiting state for spins

// Returns NULL if the pointer holds a reserved sentinel status (≤ HMCST_STATUS_M)
#define REAL_QNODE(p)   (((uintptr_t)(p) <= HMCST_STATUS_M) ? NULL : (p))

/*──────────────── TSC Clock for Timeout ──────────────────────────────*/
// Returns current timestamp counter (TSC) in cycles for timeout checks
static inline uint64_t HMCST_NOW(void) {
    unsigned hi, lo;
    __asm__ volatile("rdtsc" : "=a"(lo), "=d"(hi));
    return ((uint64_t)hi << 32) | lo;
}

/*──────────────── HMCS-T States and Types ────────────────────────────*/
// QNode states for HMCS-T protocol
#define HMCST_STATUS_R 0x0UL  /* recycled  */
#define HMCST_STATUS_W 0x1UL  /* waiting   */
#define HMCST_STATUS_U 0x2UL  /* unlocked  */
#define HMCST_STATUS_A 0x3UL  /* abandoned */
#define HMCST_STATUS_M 0x4UL  /* impatient */

// Boolean type for acquisition results
typedef vbool_t    hmcst_bool_t;
#define HMCST_SUCCESS  ((hmcst_bool_t)1)  // Successful lock acquisition
#define HMCST_TIMEOUT  ((hmcst_bool_t)0)  // Timeout during acquisition

/*──────────────── Debugging Utilities ────────────────────────────────*/
// Debugging macros for tracing lock operations (enabled with HMCST_DBG)
#ifdef HMCST_DBG
# include <stdio.h>
# include <pthread.h>
// Maps state values to human-readable names
static inline const char *st_name(uint64_t s) {
    switch (s) {
    case HMCST_STATUS_R:         return "R";
    case HMCST_STATUS_W:         return "W";
    case HMCST_STATUS_U:         return "U";
    case HMCST_STATUS_A:         return "A";
    case HMCST_STATUS_M:         return "M";
    case HMCLOCK_COHORT_START:   return "C";
    case HMCLOCK_ACQUIRE_PARENT: return "P";
    default:                     return "?";
    }
}
#define TID() ((unsigned long)pthread_self())
#define DBG_ACQ(q,L,prv,new,pred,res) \
    fprintf(stderr,"[%lu][ACQ] L=%d q=%p %s→%s pred=%p → %s\n", \
            TID(),(int)(L),(void*)(q),st_name(prv),st_name(new),(void*)(pred),(res))
#define DBG_REL(q,pass,succ) \
    fprintf(stderr,"[%lu][REL] q=%p pass=%s succ=%p\n", \
            TID(),(void*)(q),st_name(pass),(void*)(succ))
#define DBG_ABND_START(q,lvl) \
    fprintf(stderr,"[%lu][ABND] START lvl=%d q=%p\n", TID(), (int)(lvl),(void*)(q))
#define DBG_ABND_FWD(q,lvl,succ) \
    fprintf(stderr,"[%lu][ABND][FWD] lvl=%d q=%p → succ=%p\n", \
            TID(), (int)(lvl),(void*)(q),(void*)(succ))
#define DBG_ABND_REV(q,lvl,node) \
    fprintf(stderr,"[%lu][ABND][REV] lvl=%d node=%p\n", \
            TID(), (int)(lvl),(void*)(node))
#define DBG_ABND_END(q) \
    fprintf(stderr,"[%lu][ABND] END q=%p\n", TID(), (void*)(q))
#define DBG_INIT(q,st) \
    fprintf(stderr,"[%lu][INIT] q=%p status=%s\n", TID(), (void*)(q), st_name(st))
#define DBG_ENQ(q,L,pred) \
    fprintf(stderr,"[%lu][ENQ] L=%d q=%p pred=%p\n", TID(), (int)(L), (void*)(q), (void*)(pred))
#define DBG_SPIN(q,L,st,reason) /* Disabled to reduce noise */
#define DBG_STATUS(q,L,st) \
    fprintf(stderr,"[%lu][STATUS] L=%d q=%p status=%s\n", TID(), (int)(L), (void*)(q), st_name(st))
#define DBG_CAS(q,L,field,exp,des,res) \
    fprintf(stderr,"[%lu][CAS] L=%d q=%p %s: %p→%p (%s)\n", TID(), (int)(L), (void*)(q), (field), (void*)(exp), (void*)(des), (res))
#define DBG_XCHG(q,L,field,old,new) \
    fprintf(stderr,"[%lu][XCHG] L=%d q=%p %s: %p→%p\n", TID(), (int)(L), (void*)(q), (field), (void*)(old), (void*)(new))
#define DBG_REL_HELPER(q,L,action,succ,st) \
    fprintf(stderr,"[%lu][REL_HELPER] L=%d q=%p %s succ=%p status=%s\n", TID(), (int)(L), (void*)(q), (action), (void*)(succ), st_name(st))
#define DBG_REL_REAL(q,L,action,st) \
    fprintf(stderr,"[%lu][REL_REAL] L=%d q=%p %s status=%s\n", TID(), (int)(L), (void*)(q), (action), st_name(st))
#define DBG_ABND_REAL(q,L,action,succ,st) \
    fprintf(stderr,"[%lu][ABND_REAL] L=%d q=%p %s succ=%p status=%s\n", TID(), (int)(L), (void*)(q), (action), (void*)(succ), st_name(st))
#else
# define DBG_ACQ(...)
# define DBG_REL(...)
# define DBG_ABND_START(...)
# define DBG_ABND_FWD(...)
# define DBG_ABND_REV(...)
# define DBG_ABND_END(...)
# define DBG_INIT(...)
# define DBG_ENQ(...)
# define DBG_SPIN(...)
# define DBG_STATUS(...)
# define DBG_CAS(...)
# define DBG_XCHG(...)
# define DBG_REL_HELPER(...)
# define DBG_REL_REAL(...)
# define DBG_ABND_REAL(...)
#endif

/*──────────────── QNode Initialization ───────────────────────────────*/
// Initializes a QNode to recycled state for reuse
static inline void hmcst_qnode_init(hmcs_node_t *n) {
    DBG_INIT(n, HMCST_STATUS_R);
    vatomic64_write_rlx(&n->status, HMCST_STATUS_R);
    vatomicptr_write_rlx(&n->next, NULL);
}

/*──────────────── Function Declarations ──────────────────────────────*/
// Forward declarations for lock management functions
static inline void hmcst_release_helper(hmcslock_t *lock, hmcs_node_t *qnode, uint64_t pass_val);
static inline void hmcst_release_real(hmcslock_t *lock, hmcs_node_t *qnode, size_t depth);
static inline void hmcst_abandon(hmcslock_t *lock, hmcs_node_t *qn, size_t levels);
static inline void hmcst_abandon_real(hmcslock_t *lock, hmcs_node_t *qn, size_t depth);
static inline hmcst_bool_t hmcst_acquire(hmcslock_t *lock, hmcs_node_t *qn, size_t levels, uint64_t timeout_ns);
static inline hmcst_bool_t hmcst_acquire_real(hmcslock_t *lock, hmcs_node_t *qn, size_t depth, uint64_t timeout_ns);

/*──────────────── Lock Acquisition ───────────────────────────────────*/
// Acquires a hierarchical lock across specified levels with timeout
// Handles cohort inheritance, parent passing, and retries
static inline hmcst_bool_t
hmcst_acquire(hmcslock_t *lock, hmcs_node_t *qn, size_t levels, uint64_t timeout_ns)
{
    // Check current QNode state
    uint64_t prv = vatomic64_read_acq(&qn->status);

    // Case 1: Already cohort leader
    if (prv == HMCLOCK_COHORT_START) {
        DBG_ACQ(qn, levels, prv, prv, NULL, "inherited_C");
        if (levels > 1 && lock->parent) {
            if (hmcst_acquire(lock->parent, &lock->parent->qnode, levels - 1, timeout_ns) == HMCST_TIMEOUT)
                return HMCST_TIMEOUT;
        }
        return HMCST_SUCCESS;
    }
    
    // Case 2: Lock passed from parent
    if (prv == HMCLOCK_ACQUIRE_PARENT) {
        DBG_ACQ(qn, levels, prv, HMCLOCK_COHORT_START, NULL, "inherited_P");
        vatomic64_write_rel(&qn->status, HMCLOCK_COHORT_START);
        DBG_STATUS(qn, levels, HMCLOCK_COHORT_START);
        if (levels > 1 && lock->parent) {
            if (hmcst_acquire(lock->parent, &lock->parent->qnode, levels - 1, timeout_ns) == HMCST_TIMEOUT) {
                hmcst_abandon(lock, qn, levels);
                return HMCST_TIMEOUT;
            }
        }
        return HMCST_SUCCESS;
    }
    
    // Case 3: Unlocked after abandonment
    if (prv == HMCST_STATUS_U) {
        DBG_ACQ(qn, levels, prv, HMCLOCK_COHORT_START, NULL, "unlocked_after_abandon");
        vatomic64_write_rel(&qn->status, HMCLOCK_COHORT_START);
        DBG_STATUS(qn, levels, HMCLOCK_COHORT_START);
        if (levels > 1 && lock->parent) {
            if (hmcst_acquire(lock->parent, &lock->parent->qnode, levels - 1, timeout_ns) == HMCST_TIMEOUT) {
                hmcst_abandon(lock, qn, levels);
                return HMCST_TIMEOUT;
            }
        }
        return HMCST_SUCCESS;
    }
    
    // Case 4: Retry after abandonment or recycled state
    if (prv == HMCST_STATUS_A || prv == HMCST_STATUS_R) {
        uint64_t t0 = HMCST_NOW();
        if (vatomic64_cmpxchg(&qn->status, prv, HMCST_STATUS_W) == prv) {
            // Successfully reused QNode
            DBG_ACQ(qn, levels, prv, HMCST_STATUS_W, NULL, "reentry_success");
            if (hmcst_acquire_real(lock, qn, levels, timeout_ns) == HMCST_TIMEOUT)
                return HMCST_TIMEOUT;
            // Escalate to parent for multi-level locks
            if (levels > 1 && lock->parent) {
                if (hmcst_acquire(lock->parent, &lock->parent->qnode, levels - 1, timeout_ns) == HMCST_TIMEOUT) {
                    hmcst_abandon(lock, qn, levels);
                    return HMCST_TIMEOUT;
                }
            }
            // Mark as cohort leader
            vatomic64_write_rel(&qn->status, HMCLOCK_COHORT_START);
            DBG_STATUS(qn, levels, HMCLOCK_COHORT_START);
            return HMCST_SUCCESS;
        }
        // CAS failed, wait for recycled state
        while (vatomic64_read_acq(&qn->status) != HMCST_STATUS_R) {
            if (HMCST_NOW() - t0 >= timeout_ns)
                return HMCST_TIMEOUT;
            cpu_relax();
        }
        // Retry local acquisition
        vatomic64_write_rel(&qn->status, HMCST_STATUS_W);
        if (hmcst_acquire_real(lock, qn, levels, timeout_ns) == HMCST_TIMEOUT)
            return HMCST_TIMEOUT;
        // Escalate to parent
        if (levels > 1 && lock->parent) {
            if (hmcst_acquire(lock->parent, &lock->parent->qnode, levels - 1, timeout_ns) == HMCST_TIMEOUT) {
                hmcst_abandon(lock, qn, levels);
                return HMCST_TIMEOUT;
            }
        }
        // Mark as cohort leader
        vatomic64_write_rel(&qn->status, HMCLOCK_COHORT_START);
        DBG_STATUS(qn, levels, HMCLOCK_COHORT_START);
        return HMCST_SUCCESS;
    }
    
    // Case 5: Normal case, start from recycled state
    if (prv == HMCST_STATUS_R) {
        if (hmcst_acquire_real(lock, qn, levels, timeout_ns) == HMCST_TIMEOUT)
            return HMCST_TIMEOUT;
        // Escalate to parent
        if (levels > 1 && lock->parent) {
            if (hmcst_acquire(lock->parent, &lock->parent->qnode, levels - 1, timeout_ns) == HMCST_TIMEOUT) {
                hmcst_abandon(lock, qn, levels);
                return HMCST_TIMEOUT;
            }
        }
        // Mark as cohort leader
        vatomic64_write_rel(&qn->status, HMCLOCK_COHORT_START);
        DBG_STATUS(qn, levels, HMCLOCK_COHORT_START);
        return HMCST_SUCCESS;
    }
    
    // Case 6: Unexpected state, reset and retry
    DBG_ACQ(qn, levels, prv, prv, NULL, "retry_default");
    vatomic64_write_rel(&qn->status, HMCST_STATUS_R);
    return hmcst_acquire(lock, qn, levels, timeout_ns);
}

/*──────────────── Local Lock Acquisition ─────────────────────────────*/
// Acquires a local MCS lock at a specific level with timeout
// Implements MCS queue enqueuing, fast-path, and impatient handshake
static inline hmcst_bool_t
hmcst_acquire_real(hmcslock_t *lock, hmcs_node_t *qn, size_t depth, uint64_t timeout_ns)
{
    // Step A: Set QNode to waiting state
    vatomic64_write_rel(&qn->status, HMCST_STATUS_W);
    DBG_XCHG(qn, depth, "status", vatomic64_read_rlx(&qn->status), HMCST_STATUS_W);
    uint64_t t0 = HMCST_NOW();

    // Step B: Enqueue in MCS queue
    hmcs_node_t *pred = (hmcs_node_t*)vatomicptr_xchg(&lock->lock, qn);
    DBG_ENQ(qn, depth, pred);

    // Step C: Recycle recycled predecessor without next
    while (pred && vatomic64_read_rlx(&pred->status) == HMCST_STATUS_R && vatomicptr_read_rlx(&pred->next) == NULL) {
        if (vatomicptr_cmpxchg(&lock->lock, pred, qn) == pred) {
            DBG_CAS(qn, depth, "lock", pred, qn, "recycled");
            pred = NULL;
            break;
        }
        pred = (hmcs_node_t*)vatomicptr_xchg(&lock->lock, qn);
        DBG_ENQ(qn, depth, pred);
    }

    // Step D: Fast-path if queue is empty
    if (!pred) {
        DBG_ACQ(qn, depth, HMCST_STATUS_W, HMCST_STATUS_U, NULL, "fast_path");
        vatomic64_write_rel(&qn->status, HMCST_STATUS_U);
        return HMCST_SUCCESS;
    }

    // Step E: Impatient handshake
    hmcs_node_t *oldn = (hmcs_node_t*)vatomicptr_cmpxchg(&pred->next, (hmcs_node_t*)(uintptr_t)HMCST_STATUS_M, qn);
    if (oldn == (hmcs_node_t*)(uintptr_t)HMCST_STATUS_M) {
        // Predecessor yields immediately
        vatomic64_write_rel(&pred->status, HMCST_STATUS_R);
        vatomic64_write_rel(&qn->status, HMCST_STATUS_U);
        DBG_ACQ(qn, depth, HMCST_STATUS_W, HMCST_STATUS_U, pred, "impatient_inherit");
        return HMCST_SUCCESS;
    }
    // Normal linking
    vatomicptr_write_rel(&pred->next, qn);

    // Step F: Spin with timeout
    if (timeout_ns == 0) {
        // Try-lock case: no waiting
        vatomic64_write_rel(&qn->status, HMCST_STATUS_A);
        DBG_CAS(qn, depth, "status", HMCST_STATUS_W, HMCST_STATUS_A, "trylock_failed");
        hmcst_abandon(lock, qn, depth);
        return HMCST_TIMEOUT;
    }

    while (vatomic64_read_acq(&qn->status) == HMCST_STATUS_W) {
        if (HMCST_NOW() - t0 >= timeout_ns) {
            if (vatomic64_cmpxchg(&qn->status, HMCST_STATUS_W, HMCST_STATUS_A) == HMCST_STATUS_W) {
                // Timeout, abandon lock
                DBG_CAS(qn, depth, "status", HMCST_STATUS_W, HMCST_STATUS_A, "timeout");
                hmcst_abandon(lock, qn, depth);
                return HMCST_TIMEOUT;
            }
            // Lock acquired during abandon
            if (vatomic64_read_acq(&qn->status) == HMCST_STATUS_U) {
                DBG_ACQ(qn, depth, HMCST_STATUS_W, HMCST_STATUS_U, NULL, "unlocked_during_abandon");
                return HMCST_SUCCESS;
            }
        }
        cpu_relax();
    }

    // Step G: Local acquisition succeeded
    DBG_ACQ(qn, depth, HMCST_STATUS_W, vatomic64_read_rlx(&qn->status), NULL, "success_real");
    return HMCST_SUCCESS;
}

/*──────────────── Lock Abandon Wrapper ───────────────────────────────*/
// Abandons a lock acquisition after timeout, cleaning up QNode state
static inline void hmcst_abandon(hmcslock_t *lock, hmcs_node_t *qn, size_t levels)
{
    DBG_ABND_START(qn, levels);

    // Step 1: Read current QNode state
    uint64_t st0 = vatomic64_read_acq(&qn->status);

    // Step 2: Lock acquired during abandon
    if (st0 == HMCST_STATUS_U) {
        DBG_ACQ(qn, levels, st0, HMCLOCK_COHORT_START, NULL, "unlocked_during_abandon");
        vatomic64_write_rel(&qn->status, HMCLOCK_COHORT_START);
        if (levels > 1 && lock->parent) {
            if (hmcst_acquire(lock->parent, &lock->parent->qnode, levels - 1, 0) == HMCST_TIMEOUT) {
                hmcst_abandon(lock->parent, &lock->parent->qnode, levels - 1);
            }
        }
        DBG_ABND_END(qn);
        return;
    }

    // Step 3: Terminal state, no further action
    if (st0 != HMCST_STATUS_W) {
        DBG_ABND_REAL(qn, levels, "terminal_status", NULL, st0);
        DBG_ABND_END(qn);
        return;
    }

    // Step 4: Transition from waiting to abandoned
    uint64_t old = vatomic64_cmpxchg(&qn->status, HMCST_STATUS_W, HMCST_STATUS_A);
    DBG_CAS(qn, levels, "status", HMCST_STATUS_W, HMCST_STATUS_A, old == HMCST_STATUS_W ? "abandon" : "failed");

    // Step 5: Lock acquired during CAS
    if (old == HMCST_STATUS_U) {
        DBG_ACQ(qn, levels, old, HMCLOCK_COHORT_START, NULL, "unlocked_during_abandon");
        vatomic64_write_rel(&qn->status, HMCLOCK_COHORT_START);
        if (levels > 1 && lock->parent) {
            if (hmcst_acquire(lock->parent, &lock->parent->qnode, levels - 1, 0) == HMCST_TIMEOUT) {
                hmcst_abandon(lock->parent, &lock->parent->qnode, levels - 1);
            }
        }
        DBG_ABND_END(qn);
        return;
    }

    // Step 6: Other terminal state, exit
    if (old != HMCST_STATUS_W) {
        DBG_ABND_REAL(qn, levels, "cas_failed_terminal", NULL, old);
        DBG_ABND_END(qn);
        return;
    }

    // Step 7: Forward abandon to higher levels or recycle
    if (levels > 1) {
        DBG_ABND_REAL(qn, levels, "forward_pass", NULL, HMCST_STATUS_A);
        hmcst_abandon_real(lock, qn, levels);
    } else {
        // Leaf level: recycle QNode
        vatomic64_write_rel(&qn->status, HMCST_STATUS_R);
        DBG_STATUS(qn, levels, HMCST_STATUS_R);
    }

    DBG_ABND_END(qn);
}

/*──────────────── Hierarchical Abandon ───────────────────────────────*/
// Propagates abandonment across hierarchical levels
static inline void
hmcst_abandon_real(hmcslock_t *lock, hmcs_node_t *qn, size_t depth)
{
    DBG_ABND_START(qn, depth);

    // Step 1: Navigate to leaf lock (level 1)
    hmcslock_t *leaf = lock;
    size_t lvl = depth;
    while (lvl > 1 && leaf->parent) {
        leaf = leaf->parent;
        lvl--;
    }

    // Step 2: Build arrays of locks and QNodes from leaf to current level
    hmcslock_t  *locks_arr[depth];
    hmcs_node_t *qnodes_arr[depth];
    locks_arr[0]  = leaf;
    qnodes_arr[0] = &leaf->qnode;
    for (size_t d = 1; d < depth; ++d) {
        locks_arr[d]  = locks_arr[d-1]->parent;
        qnodes_arr[d] = &locks_arr[d]->qnode;
        DBG_ABND_REAL(qn, depth, "build_arrays", qnodes_arr[d], vatomic64_read_rlx(&qnodes_arr[d]->status));
    }

    // Step 3: Search for first waiting successor across levels
    size_t found = 0;
    for (size_t d = 0; d < depth; ++d) {
        hmcslock_t  *L   = locks_arr[d];
        hmcs_node_t *Q   = qnodes_arr[d];
        hmcs_node_t *succ = REAL_QNODE(vatomicptr_read_acq(&Q->next));
        DBG_ABND_REAL(Q, d+1, "check_succ", succ, succ ? vatomic64_read_rlx(&succ->status) : 0);

        if (!succ) {
            // If QNode is cohort leader, clear tail
            if (vatomic64_read_rlx(&Q->status) == HMCLOCK_COHORT_START) {
                hmcs_node_t *tail = vatomicptr_read_rlx(&L->lock);
                if (tail == Q && vatomicptr_cmpxchg(&L->lock, Q, NULL) == Q) {
                    DBG_CAS(Q, d+1, "lock", Q, NULL, "relinquish");
                }
            }
            continue;
        }

        // Traverse chain to find waiting successor
        while (succ) {
            uint64_t st = vatomic64_read_acq(&succ->status);
            DBG_ABND_REAL(Q, d+1, "succ_status", succ, st);

            if (st == HMCST_STATUS_W) {
                // Pass lock to waiting successor
                DBG_ABND_FWD(Q, d+1, succ);
                vatomic64_xchg(&succ->status, HMCLOCK_ACQUIRE_PARENT);
                DBG_XCHG(succ, d+1, "status", HMCST_STATUS_W, HMCLOCK_ACQUIRE_PARENT);
                found = d+1;
                goto rev_real;
            }

            // Skip abandoned or impatient successors
            succ = REAL_QNODE(vatomicptr_read_acq(&succ->next));
            DBG_ABND_REAL(Q, d+1, "next_succ", succ, succ ? vatomic64_read_rlx(&succ->status) : 0);
        }
    }

rev_real:
    // Step 4: If no waiting successor, clear lock at root
    if (!found) {
        found = depth;
        hmcslock_t  *L = locks_arr[depth-1];
        hmcs_node_t *Q = qnodes_arr[depth-1];
        if (vatomicptr_read_rlx(&L->lock) == Q && vatomicptr_cmpxchg(&L->lock, Q, NULL) == Q) {
            DBG_CAS(Q, depth, "lock", Q, NULL, "relinquish_root");
        }
    }

    // Step 5: Descend levels, notify successors, and recycle
    for (int d = (int)found - 1; d >= 0; --d) {
        hmcslock_t  *L = locks_arr[d];
        hmcs_node_t *Q = qnodes_arr[d];
        DBG_ABND_REV(Q, d+1, Q);
        DBG_ABND_REAL(Q, d+1, "release_parent", NULL, vatomic64_read_rlx(&Q->status));

        // Notify successors with parent status
        hmcst_release_helper(L, Q, HMCLOCK_ACQUIRE_PARENT);

        // Recycle abandoned or parent-passed successors
        hmcs_node_t *s = REAL_QNODE(vatomicptr_read_acq(&Q->next));
        while (s) {
            uint64_t st = vatomic64_read_acq(&s->status);
            if (st == HMCLOCK_ACQUIRE_PARENT || st == HMCST_STATUS_M || st == HMCST_STATUS_A) {
                vatomic64_write_rel(&s->status, HMCST_STATUS_R);
                vatomicptr_write_rel(&s->next, NULL);
                DBG_STATUS(s, d+1, HMCST_STATUS_R);
                DBG_ABND_REV(Q, d+1, s);
            }
            s = REAL_QNODE(vatomicptr_read_acq(&s->next));
            DBG_ABND_REAL(Q, d+1, "next_recycle", s, s ? vatomic64_read_rlx(&s->status) : 0);
        }
    }

    // Step 6: Recycle initial QNode if still abandoned
    if (vatomic64_read_rlx(&qn->status) == HMCST_STATUS_A) {
        vatomic64_write_rel(&qn->status, HMCST_STATUS_R);
        DBG_STATUS(qn, depth, HMCST_STATUS_R);
    }

    DBG_ABND_END(qn);
}

/*──────────────── Lock Release Helper ────────────────────────────────*/
// Notifies successors and recycles abandoned QNodes during release
static inline void
hmcst_release_helper(hmcslock_t *lock, hmcs_node_t *qnode, uint64_t pass_val)
{
    DBG_REL_HELPER(qnode, 0, "enter", NULL, pass_val);
    hmcs_node_t *succ = REAL_QNODE(vatomicptr_read_acq(&qnode->next));
    DBG_REL(qnode, pass_val, succ);
    hmcs_node_t *prev = qnode;
    
    // Track visited nodes to prevent cycles
    hmcs_node_t *visited[32]; // Adjustable size
    size_t visited_count = 0;
    visited[visited_count++] = qnode;
    
    // Traverse successors
    while (succ) {
        // Check for cycles
        for (size_t i = 0; i < visited_count; i++) {
            if (visited[i] == succ) {
                DBG_REL_HELPER(qnode, 0, "cycle_detected", succ, vatomic64_read_acq(&succ->status));
                succ = NULL;
                break;
            }
        }
        if (!succ) break;
        
        visited[visited_count++] = succ;
        if (visited_count >= 32) {
            DBG_REL_HELPER(qnode, 0, "visited_overflow", succ, 0);
            break; // Prevent overflow
        }
        
        uint64_t st = vatomic64_read_acq(&succ->status);
        DBG_REL_HELPER(qnode, 0, "check_succ", succ, st);
        if (st == HMCST_STATUS_W) {
            // Pass lock to waiting successor
            vatomic64_xchg(&succ->status, pass_val);
            DBG_XCHG(succ, 0, "status", st, pass_val);
            DBG_REL_HELPER(qnode, 0, "pass_to_W", succ, pass_val);
            break;
        }
        if (st == HMCST_STATUS_A) {
            // Recycle abandoned successor
            DBG_REL_HELPER(qnode, 0, "abandoned", succ, st);
            hmcs_node_t *next = REAL_QNODE(vatomicptr_read_acq(&succ->next));
            vatomic64_write_rel(&succ->status, HMCST_STATUS_R);
            vatomicptr_write_rel(&succ->next, NULL);
            DBG_XCHG(succ, 0, "next", next, NULL);
            DBG_REL_HELPER(qnode, 0, "recycle_abandoned", succ, HMCST_STATUS_R);
            prev = succ;
            succ = next;
            DBG_REL_HELPER(qnode, 0, "next_abandoned", succ, succ ? vatomic64_read_rlx(&succ->status) : 0);
            continue;
        }
        if (st == HMCLOCK_COHORT_START) {
            // Avoid cycle with cohort leader
            DBG_REL_HELPER(qnode, 0, "cohort_leader", succ, st);
            succ = NULL;
            break;
        }
        // Handle unexpected states
        DBG_REL_HELPER(qnode, 0, "unexpected", succ, st);
        prev = succ;
        succ = REAL_QNODE(vatomicptr_read_acq(&succ->next));
        DBG_REL_HELPER(qnode, 0, "next_unexpected", succ, succ ? vatomic64_read_rlx(&succ->status) : 0);
    }
    
    // No successor found, attempt to clear lock
    if (!succ) {
        DBG_REL_HELPER(qnode, 0, "no_succ", NULL, 0);
        if (vatomicptr_cmpxchg(&lock->lock, prev, NULL) == prev) {
            DBG_CAS(qnode, 0, "lock", prev, NULL, "clear_tail");
        } else {
            // Brief spin to check for late successors
            const uint64_t SPIN_NS = 1000000;
            uint64_t t0 = HMCST_NOW();
            do {
                cpu_relax();
                succ = REAL_QNODE(vatomicptr_read_acq(&qnode->next));
            } while (!succ && (HMCST_NOW() - t0) < SPIN_NS);
            if (succ && vatomic64_read_acq(&succ->status) == HMCST_STATUS_W) {
                vatomic64_xchg(&succ->status, pass_val);
                DBG_XCHG(succ, 0, "status", HMCST_STATUS_W, pass_val);
                DBG_REL_HELPER(qnode, 0, "pass_to_W_late", succ, pass_val);
            } else {
                if (vatomicptr_cmpxchg(&lock->lock, prev, NULL) == prev) {
                    DBG_CAS(qnode, 0, "lock", prev, NULL, "clear_tail_final");
                }
            }
        }
    }
    
    // Recycle remaining abandoned nodes
    hmcs_node_t *cur = prev;
    while (cur != qnode && cur != NULL) {
        hmcs_node_t *next = REAL_QNODE(vatomicptr_read_rlx(&cur->next));
        vatomic64_write_rel(&cur->status, HMCST_STATUS_R);
        vatomicptr_write_rel(&cur->next, NULL);
        DBG_REL_HELPER(qnode, 0, "recycle", cur, HMCST_STATUS_R);
        cur = next;
    }
    
    DBG_REL_HELPER(qnode, 0, "exit", NULL, 0);
}

/*──────────────── Local Lock Release ─────────────────────────────────*/
// Releases a lock at a specific level, handling cohort thresholds
static inline void
hmcst_release_real(hmcslock_t *lock, hmcs_node_t *qnode, size_t depth)
{
    DBG_REL_REAL(qnode, depth, "enter", vatomic64_read_rlx(&qnode->status));
    uint64_t c = vatomic64_read_rlx(&qnode->status);
    // Check cohort threshold for local passing
    if (c < lock->threshold) {
        hmcs_node_t *succ = REAL_QNODE(vatomicptr_read_acq(&qnode->next));
        DBG_REL_REAL(qnode, depth, "check_local", c);
        if (succ) {
            DBG_REL_REAL(qnode, depth, "pass_local", c + 1);
            hmcst_release_helper(lock, qnode, c + 1);
            return;
        }
    }
    // Escalate to parent if applicable
    if (lock->parent && depth > 0) {
        DBG_REL_REAL(qnode, depth, "escalate_parent", c);
        hmcst_release_real(lock->parent, &lock->parent->qnode, depth - 1);
        DBG_REL_REAL(qnode, depth, "notify_succ", HMCLOCK_ACQUIRE_PARENT);
        hmcst_release_helper(lock, qnode, HMCLOCK_ACQUIRE_PARENT);
        return;
    }
    // Start new cohort
    DBG_REL_REAL(qnode, depth, "new_cohort", HMCLOCK_COHORT_START);
    hmcst_release_helper(lock, qnode, HMCLOCK_COHORT_START);
}

/*──────────────── Hierarchical Lock Release ──────────────────────────*/
// Releases a hierarchical lock across specified levels
static inline void
hmcst_release(hmcslock_t *lock, hmcs_node_t *qnode, size_t levels)
{
    DBG_REL(qnode, HMCST_STATUS_R, NULL);
    DBG_REL_REAL(qnode, levels - 1, "release_real", vatomic64_read_rlx(&qnode->status));
    // Release lock at specified level
    hmcst_release_real(lock, qnode, levels - 1);
    // Recycle QNode
    vatomic64_write_rel(&qnode->status, HMCST_STATUS_R);
    vatomicptr_write_rel(&qnode->next, NULL);
    DBG_STATUS(qnode, levels, HMCST_STATUS_R);
    DBG_REL(qnode, HMCST_STATUS_R, NULL);
}
#endif /* HMCSTLOCK_H */
