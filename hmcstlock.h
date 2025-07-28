#ifndef HMCSTLOCK_H
#define HMCSTLOCK_H

/*──────────────── Dependencies and Base Definitions ───────────────────*/
#include "hmcslock.h"
#include <assert.h>
#include <stdatomic.h>
#include <time.h>
#include <stdint.h>
#include <sched.h>
#include <stdio.h>
#include <string.h>

#define cpu_relax() sched_yield()

/*──────────────── PHASE 1: Correct Constants (Paper Section 3) ────────*/
// Status values selon le papier (page 5-6)
#undef HMCST_STATUS_R
#undef HMCST_STATUS_A
#undef HMCST_STATUS_W
#undef HMCST_STATUS_P
#undef HMCST_STATUS_M
#undef HMCST_STATUS_U
#undef HMCST_STATUS_C      

#define HMCST_STATUS_C 1UL
#define HMCST_STATUS_V_START 2UL
#define HMCST_STATUS_R (~0UL - 3)
#define HMCST_STATUS_A (~0UL - 2)
#define HMCST_STATUS_W (~0UL - 1)
#define HMCST_STATUS_P (~0UL)
#define HMCST_STATUS_M ((hmcs_node_t*)(~0UL - 4))
#define HMCST_STATUS_U (~0UL - 5)

// Sentinel values
#define HMCST_INVALID_PTR ((hmcs_node_t*)(uintptr_t)0xDEADBEEF)

// Result type
typedef int hmcst_bool_t;
#define HMCST_SUCCESS  1
#define HMCST_TIMEOUT  0



// On utilise un type de retour pour être plus explicite
typedef enum {
    PEER_FOUND,
    PEER_NOT_FOUND
} find_peer_result_t;


// TSC for timeout
static inline uint64_t HMCST_NOW(void) {
    unsigned hi, lo;
    __asm__ volatile("rdtsc" : "=a"(lo), "=d"(hi));
    return ((uint64_t)hi << 32) | lo;
}

// Convert pointer to real node (handle M sentinel)
#define REAL_QNODE(p) (((uintptr_t)(p) == HMCST_STATUS_M) ? NULL : (p))

/*──────────────── Debug Macros ────────────────────────────────────────*/
#ifdef HMCST_DBG
#include <pthread.h>
static inline const char *st_name(uint64_t s) {
    // First, check for the special high-value transient states.
    if (s == HMCST_STATUS_R) return "R";
    if (s == HMCST_STATUS_A) return "A";
    if (s == HMCST_STATUS_W) return "W";
    if (s == HMCST_STATUS_P) return "P";
    if (s == HMCST_STATUS_M) return "M";
    if (s == HMCST_STATUS_U) return "U";
    // Next, check for the simple lock-holding states.
    if (s == HMCST_STATUS_C) return "C";
    
    // If it's none of the above, it must be a V value (local passing count).
    // The check `s < HMCST_STATUS_R` ensures we don't misinterpret a huge
    // number as a V value if we add more transient states later.
    if (s >= HMCST_STATUS_V_START && s < HMCST_STATUS_R) {
        // Using a thread-local buffer is safer in multi-threaded logging
        // than a static buffer to prevent garbled output.
        static __thread char buf[32]; 
        
        // The V counter starts at V_START=2. So a status of 2 is "V1", 3 is "V2", etc.
        snprintf(buf, sizeof(buf), "V%lu", s - HMCST_STATUS_V_START + 1);
        return buf;
    }

    // If we reach here, the status is unknown.
    return "?";
}

// The ptr_name function remains the same, but we need to ensure HMCST_STATUS_M
// is correctly handled if its value changes.
static inline const char *ptr_name(void *p) {
    static __thread char buf[32]; // Use thread-local here as well for safety
    if (p == NULL) return "NULL";
    // HMCST_STATUS_M is a pointer cast, so compare it directly.
    if (p == HMCST_STATUS_M) return "M"; 
    snprintf(buf, sizeof(buf), "%p", p);
    return buf;
}

#define TID() ((unsigned long)pthread_self() & 0xFFFF)
// FIX: Renamed DBG to HMCST_LOG to avoid redefinition conflicts.
#define HMCST_LOG(fmt, ...) fprintf(stderr, "[HMCST][T%04lx][%s:%d] " fmt, TID(), __func__, __LINE__, ##__VA_ARGS__)
#define HMCST_LOG_STATE(msg, qn) HMCST_LOG("%s: qnode=%p status=%s next=%s\n", msg, (void*)(qn), st_name(vatomic64_read_rlx(&(qn)->status)), ptr_name(vatomicptr_read_rlx(&(qn)->next)))
#define HMCST_LOG_LOCK(msg, lock) HMCST_LOG("%s: lock=%p tail=%s\n", msg, (void*)(lock), ptr_name(vatomicptr_read_rlx(&(lock)->lock)))
#define HMCST_LOG_TIME(msg, start, timeout) HMCST_LOG("%s: elapsed=%lu ns, timeout=%lu ns\n", msg, HMCST_NOW() - start, timeout)
#else
#define HMCST_LOG(...)
#define HMCST_LOG_STATE(...)
#define HMCST_LOG_LOCK(...)
#define HMCST_LOG_TIME(...)
#endif

// Helper macro for timeout checking
#define Check_Timeout(start_time, timeout_ns) \
    ((timeout_ns > 0) && (HMCST_NOW() - start_time >= timeout_ns))

// For debugging spin loops
#ifdef DEBUG
#define DBG_SPIN(...) do { \
    static uint64_t spin_count = 0; \
    if ((++spin_count % 10000) == 0) { \
        DBG(__VA_ARGS__); \
    } \
} while(0)
#else
#define DBG_SPIN(...)
#endif

#endif /* HMCSTLOCK_CORRECTED_H */

typedef enum {
    PASS_FAILED_EMPTY,
    /** The lock was successfully passed to a waiting successor. */
    PASS_SUCCESS,
    PASS_FAILED_IMPATIENT
} pass_result_t;


/*──────────────── QNode Initialization ───────────────────────────────*/
static inline void hmcst_qnode_init(hmcs_node_t *n) {
    assert(n != NULL);
    vatomic64_write_rlx(&n->status, HMCST_STATUS_R);
    vatomicptr_write_rlx(&n->next, NULL);
}

/*──────────────── Forward Declarations ────────────────────────────────*/
static inline hmcst_bool_t hmcst_acquire(hmcslock_t *lock, hmcs_node_t *qn, size_t levels, uint64_t timeout_ns);
static inline hmcst_bool_t hmcst_acquire_real(hmcslock_t *lock, hmcs_node_t *qn, size_t depth, uint64_t timeout_ns);
static inline void hmcst_release(hmcslock_t* base_lock, hmcs_node_t* thread_qnode, int n_levels);
static inline bool hmcst_release_single_level(hmcslock_t *lock, hmcs_node_t *qnode,uint64_t status_to_pass);
static inline hmcst_bool_t hmcst_release_helper(hmcslock_t *lock, hmcs_node_t *releaser_qnode, uint64_t pass_val);
static inline void hmcst_abort(hmcslock_t* base_lock, int n_levels, hmcs_node_t* qnode, int abort_level);
static inline find_peer_result_t hmcst_scan_for_successor(hmcs_node_t* start_node, hmcs_node_t** out_found_successor, hmcs_node_t** out_last_scanned_node);
pass_result_t hmcst_find_and_pass(hmcslock_t* lock, hmcs_node_t* releaser_qnode, uint64_t status_to_pass);
static inline bool  hmcst_release_recursive(hmcslock_t* locks[], int current_level, int max_levels, hmcs_node_t* initial_qnode) ;
void hmcst_cleanup_path(hmcs_node_t* start_node, hmcs_node_t* stop_at_succ);



/*──────────────── PHASE 2: Correct Acquisition Protocol (Paper Section 3) ───*/
// One-level acquisition (Paper Section 3 - 3 cases)
static inline hmcst_bool_t
hmcst_acquire_real(hmcslock_t *lock, hmcs_node_t *qn, size_t depth, uint64_t timeout_ns)
{
    assert(lock != NULL);
    assert(qn != NULL);
    
    uint64_t start_time = HMCST_NOW();
    
    HMCST_LOG("ENTER: depth=%zu timeout=%lu ns\n", depth, timeout_ns);
    HMCST_LOG_STATE("initial", qn);
    HMCST_LOG_LOCK("initial", lock);
    
    // Step 1: SWAP W into status (Paper page 7: "t begins by SWAPing W")
    uint64_t prev_status = vatomic64_xchg(&qn->status, HMCST_STATUS_W);
    HMCST_LOG("SWAP: status %s -> W\n", st_name(prev_status));
    HMCST_LOG_STATE("after swap", qn);
    
    // Handle 3 cases for one-level HMCS-T according to paper Section 3
    switch (prev_status) {
    case HMCST_STATUS_R:
        // Case 1: Recycled node - normal MCS enqueue (Paper page 7)
        HMCST_LOG("CASE R: recycled node, proceed to enqueue\n");
        break;
        
    case HMCST_STATUS_A:
        // Case 2: Previously abandoned (Paper page 7: "resume wait directly")
        HMCST_LOG("CASE A: previously abandoned, resume wait without enqueue\n");
        goto spin_wait;
        
    case HMCST_STATUS_U:
        // Case 3: Node unlocked after abandonment (Paper page 8: "wait for R")
        HMCST_LOG("CASE U: unlocked after abandon, must wait for R\n");
        while (vatomic64_read_acq(&qn->status) != HMCST_STATUS_R) {
            if (HMCST_NOW() - start_time >= timeout_ns) {
                // Try to revert to U
                uint64_t expected = HMCST_STATUS_W;
                if (vatomic64_cmpxchg(&qn->status, expected, HMCST_STATUS_U) == HMCST_STATUS_W) {
                    HMCST_LOG("TIMEOUT: reverted status to U\n");
                    return HMCST_TIMEOUT;
                }
                // Status changed - check if R
                if (vatomic64_read_rlx(&qn->status) == HMCST_STATUS_R) {
                    break;
                }
            }
            cpu_relax();
        }
        // Now recycled, retry (optimistic strategy)
        HMCST_LOG("GOT R: retrying acquisition\n");
        return hmcst_acquire_real(lock, qn, depth, timeout_ns);
        
    default:
        // Should not happen for one-level
        HMCST_LOG("ERROR: unexpected status %s for one-level\n", st_name(prev_status));
        assert(0 && "Invalid status for one-level HMCS-T");
        return HMCST_TIMEOUT;
    }
    
    // Step 2: MCS enqueue (Paper Figure 3)
    HMCST_LOG_LOCK("before enqueue", lock);
    hmcs_node_t *pred = (hmcs_node_t*)vatomicptr_xchg(&lock->lock, qn);
    HMCST_LOG("ENQUEUE: pred=%s, I am new tail\n", ptr_name(pred));
    HMCST_LOG_LOCK("after enqueue", lock);
    
    if (pred == NULL) {
        // Uncontended - we got the lock
        vatomic64_write_rel(&qn->status, HMCST_STATUS_U);
        HMCST_LOG("FAST PATH: no predecessor, got lock immediately\n");
        HMCST_LOG_STATE("fast path success", qn);
        return HMCST_SUCCESS;
    }
    
    // Step 3: Link to predecessor
    HMCST_LOG("LINKING: updating pred->next\n");
    vatomicptr_write_rel(&pred->next, qn);
    
spin_wait:
    // Step 4: Spin wait with timeout (Paper page 7)
    HMCST_LOG("SPIN WAIT: waiting for status change from W\n");
    uint64_t spin_count = 0;
    while (vatomic64_read_acq(&qn->status) == HMCST_STATUS_W) {
        spin_count++;
        
        if (HMCST_NOW() - start_time >= timeout_ns) {
            // Try to abandon (Paper page 7: "CAS A into status")
            uint64_t expected = HMCST_STATUS_W;
            HMCST_LOG("TIMEOUT: attempting to abandon after %lu spins\n", spin_count);
            if (vatomic64_cmpxchg(&qn->status, expected, HMCST_STATUS_A) == HMCST_STATUS_W) {
                HMCST_LOG("ABANDONED: successfully set status to A\n");
                HMCST_LOG_STATE("abandoned", qn);
                assert(0 && "Deadlock detected by spin wait timeout");
                return HMCST_TIMEOUT;
            }
            // CAS failed - we got the lock
            HMCST_LOG("ABANDON FAILED: got lock during CAS\n");
            break;
        }
        cpu_relax();
    }
    
    // Check final status
    uint64_t final_status = vatomic64_read_acq(&qn->status);
    HMCST_LOG("EXIT SPIN: final status=%s after %lu spins\n", st_name(final_status), spin_count);
    
    if (final_status == HMCST_STATUS_U) {
        HMCST_LOG("SUCCESS: got lock\n");
        return HMCST_SUCCESS;
    }
    
    HMCST_LOG("FAILED: unexpected final status %s\n", st_name(final_status));
    return HMCST_TIMEOUT;
}
static inline hmcst_bool_t
hmcst_acquire(hmcslock_t *lock, hmcs_node_t *qnode, size_t levels, uint64_t timeout_ns)
{
    assert(lock != NULL && qnode != NULL && levels > 0);
    HMCST_LOG("ENTER: levels=%zu timeout=%lu ns\n", levels, timeout_ns);
    uint64_t initial_status = vatomic64_read_rlx(&qnode->status);

    // Validation initiale prudente (conservée comme demandé)
    bool is_status_valid =
        (initial_status == HMCST_STATUS_R) ||
        (initial_status == HMCST_STATUS_A) ||   
        (initial_status == HMCST_STATUS_W) ||
        (initial_status == HMCST_STATUS_P) ||
        (initial_status == HMCST_STATUS_C) ||
        (initial_status == HMCST_STATUS_U) ||
        (initial_status >= HMCST_STATUS_V_START && initial_status < HMCST_STATUS_R);

    if (!is_status_valid) {
        HMCST_LOG("WARNING: QNode %p has invalid initial status (%lx). Forcing to R.\n",
                  (void*)qnode, initial_status);
        hmcst_qnode_init(qnode); // Force l'état à {R, NULL}
    }

  

    // =======================================================================
    // === 2. SLOW-PATH (Level-by-Level Hierarchical Acquisition) ============
    // =======================================================================

    uint64_t start_time = HMCST_NOW();
    hmcslock_t *current_lock = lock;
    hmcs_node_t *qnode_to_use = qnode;
    bool is_root_level;

    for (size_t level = 0; level < levels; ++level) {
        hmcs_node_t *current_qnode = qnode_to_use;
        hmcs_node_t *pred = NULL;
        is_root_level = (level == levels - 1);

    retry_level:
        HMCST_LOG("LEVEL %zu: lock=%p, using %s qnode (%p), is_root=%d\n", 
                  level + 1, (void*)current_lock,
                  (level == 0) ? "thread's" : "lock's", 
                  (void*)current_qnode, is_root_level);

        // ÉTAPE CRITIQUE : SWAP atomique de W dans status
        uint64_t prev_status = vatomic64_xchg(&current_qnode->status, HMCST_STATUS_W);
        HMCST_LOG("SWAP: prev_status=%s -> W\n", st_name(prev_status));

        // === CAS R (Recycled) ===
        if (prev_status == HMCST_STATUS_R) {
            HMCST_LOG("CASE R: Recycled node. Enqueueing.\n");
            vatomicptr_write_rlx(&current_qnode->next, NULL);
            pred = (hmcs_node_t *)vatomicptr_xchg(&current_lock->lock, current_qnode);
HMCST_LOG("ACQUIRE: Exchanged tail. Found predecessor: %p (status: %s)\n",
          (void*)pred, (pred ? st_name(vatomic64_read_rlx(&pred->status)) : "N/A"));

            if (pred == NULL) {
                // Pas de prédécesseur
                if (is_root_level) {
                    HMCST_LOG("Root level uncontended. Setting to U.\n");
                    vatomic64_write_rel(&current_qnode->status, HMCST_STATUS_U);
                    return HMCST_SUCCESS;
                } else {
                    HMCST_LOG("Non-root uncontended. Setting to C and proceeding.\n");
                    vatomic64_write_rel(&current_qnode->status, HMCST_STATUS_C);
                    goto next_level;
                }
            }

            HMCST_LOG("ACQUIRE L%zu: Predecessor is not NULL. Address: %p. I will now link to it.\n", 
              level + 1, (void*)pred);
            
             hmcs_node_t* old_pred_next = (hmcs_node_t*)vatomicptr_xchg(&pred->next, current_qnode);

            // Maintenant, on analyse ce qu'on a récupéré de manière atomique.
            if (old_pred_next == NULL) {
                // CAS NORMAL : Le prédécesseur n'avait pas encore agi.
                // Le lien est établi, on peut attendre en toute sécurité.
                HMCST_LOG("ACQUIRE L%zu: Link successful (pred->next was NULL). Now going to spin.\n", level + 1);
                goto spin_wait;
            }
            else if (old_pred_next == (void*)HMCST_STATUS_M) {
                // CAS DE LA COURSE GAGNÉ : Le prédécesseur était impatient et a laissé M.
                // Nous avons récupéré M et l'avons remplacé par notre qnode en une seule étape.
                // Nous héritons du verrou immédiatement !
                HMCST_LOG("ACQUIRE L%zu: Detected M in pred->next during link! Inheriting lock.\n", level + 1);
                
                // Nous sommes maintenant responsables du recyclage du prédécesseur.
                vatomic64_write_rel(&pred->status, HMCST_STATUS_R);
                
                if (is_root_level) {
                    vatomic64_write_rel(&current_qnode->status, HMCST_STATUS_U);
                    return HMCST_SUCCESS;
                } else {
                    vatomic64_write_rel(&current_qnode->status, HMCST_STATUS_C);
                    goto next_level;
                }
            }
            else {
                // Ce cas ne devrait jamais se produire dans un protocole MCS correct,
                // car seul un successeur direct devrait se lier.
                HMCST_LOG("FATAL: Unexpected value %p in pred->next during link.\n", (void*)old_pred_next);
                assert(0 && "Collision during linking phase");
                return HMCST_TIMEOUT; // Ou une autre gestion d'erreur
            }

            

        // === CAS A (Abandoned) ===
        } else if (prev_status == HMCST_STATUS_A) {
            HMCST_LOG("CASE A: Previously abandoned. Resuming spin wait.\n");
            // Ne PAS réinitialiser next, ne PAS re-enqueuer
            goto spin_wait;

        // === CAS C (Inherited ancestral lock) - Non-root seulement ===
        } else if (prev_status == HMCST_STATUS_C && !is_root_level) {
            HMCST_LOG("CASE C: Inherited lock. Reverting to C and proceeding.\n");
            vatomic64_write_rel(&current_qnode->status, HMCST_STATUS_C);
            goto next_level;

        } else { // Handles V and P cases (prev_status was HMCST_STATUS_P or HMCST_STATUS_V)
            HMCST_LOG("CASE P/V: Node is being unlocked. Waiting for it to become R.\n");
            
            while (vatomic64_read_acq(&current_qnode->status) != HMCST_STATUS_R) {
                if (Check_Timeout(start_time, timeout_ns)) {
                    // SPECIFICATION: "if t times out, it CASes the value P into q.status and aborts if CAS succeeds"
                    uint64_t expected_pv = prev_status;
                    if (vatomic64_cmpxchg(&current_qnode->status, expected_pv, HMCST_STATUS_P) == expected_pv) {
                        HMCST_LOG("TIMEOUT: CASed P into q.status while waiting for R. Aborting.\n");
                        hmcst_abort(lock, levels,&current_qnode, level+1);
                        return HMCST_TIMEOUT;
                    }
                    else {

                        HMCST_LOG("TIMEOUT CAS failed, lock granted (status is now R). Retrying level optimistically.\n");
                        goto retry_level;}
                }
                
            }
            HMCST_LOG("Node is now R. Retrying level.\n");
            goto retry_level;
        }
       

    spin_wait:
        HMCST_LOG("SPIN WAIT: Entering at level %zu.\n", level + 1);
        
        // Vérifier d'abord si notre prédécesseur est impatient (M dans next)
        if (pred && vatomicptr_read_acq(&pred->next) == (void*)HMCST_STATUS_M) {
            HMCST_LOG("SPIN WAIT: Predecessor left M in next! I inherit the lock.\n");
            // Recycler le QNode du prédécesseur
            vatomic64_write_rel(&pred->status, HMCST_STATUS_R);
            
            if (is_root_level) {
                vatomic64_write_rel(&current_qnode->status, HMCST_STATUS_U);
                return HMCST_SUCCESS;
            } else {
                vatomic64_write_rel(&current_qnode->status, HMCST_STATUS_C);
                goto next_level;
            }
        }
        
        // Attendre que notre status change
        while (1) {
            uint64_t status = vatomic64_read_acq(&current_qnode->status);
    if (status != HMCST_STATUS_W) {
        HMCST_LOG("SPIN WAIT: Status changed to %s.\n", st_name(status));

        if (status == HMCST_STATUS_U && is_root_level) {
            HMCST_LOG("SUCCESS: Got U at root level.\n");
            return HMCST_SUCCESS;
        }
        if (status >= HMCST_STATUS_V_START && status < HMCST_STATUS_R) {
            HMCST_LOG("SUCCESS: Got passing value V=%lu.\n", status);
            return HMCST_SUCCESS;
        }
        if (status == HMCST_STATUS_P) {
            HMCST_LOG("Got P. Setting to C and proceeding.\n");
            vatomic64_write_rel(&current_qnode->status, HMCST_STATUS_C);
            goto next_level;
        }
        // Si le statut est autre (ex: A), on ne fait rien et on continue la boucle.
        // On laisse la chance à la vérification de M ou à un autre changement de statut.
    }

    // --- VÉRIFICATION 2 : Marqueur Impatient ---
    if (pred && vatomicptr_read_acq(&pred->next) == (void*)HMCST_STATUS_M) {
        HMCST_LOG("SPIN WAIT: Detected M in pred->next! Inheriting lock.\n");
        vatomic64_write_rel(&pred->status, HMCST_STATUS_R);
        if (is_root_level) {
            vatomic64_write_rel(&current_qnode->status, HMCST_STATUS_U);
            return HMCST_SUCCESS;
        } else {
            vatomic64_write_rel(&current_qnode->status, HMCST_STATUS_C);
            goto next_level;
        }
    }


            // Vérifier le timeout
    if (Check_Timeout(start_time, timeout_ns)) {
                uint64_t expected = HMCST_STATUS_W;
                if (vatomic64_cmpxchg(&current_qnode->status, expected, HMCST_STATUS_A) == expected) {
                    HMCST_LOG("TIMEOUT: Successfully abandoned at level %zu.\n", level + 1);
                    hmcst_abort(lock, levels,qnode, level+1);
                    return HMCST_TIMEOUT;
                }
                // CAS a échoué : on a reçu le lock entre temps
                HMCST_LOG("TIMEOUT CAS failed. Lock was granted.\n");
            }
            
            cpu_relax();
        }

    next_level:
        if (level < levels - 1) {
            qnode_to_use = &current_lock->qnode;
            current_lock = current_lock->parent;
        }
    }

    HMCST_LOG("SUCCESS: Acquired all %zu levels.\n", levels);
    return HMCST_SUCCESS;
}

// =======================================================================
// == FONCTION DE LIBÉRATION GÉNÉRIQUE POUR UN SEUL NIVEAU ===============
// =======================================================================
// Prend en paramètre le statut à passer au successeur (ex: U, P, V+1).
/*
bool hmcst_release_single_level(hmcslock_t* lock, hmcs_node_t* releaser_qnode, uint64_t status_to_pass) {

     // [DEBUG] Valider les pointeurs d'entrée.
    assert(lock != NULL && "hmcst_release_single_level: lock pointer is NULL");
    assert(releaser_qnode != NULL && "hmcst_release_single_level: releaser_qnode pointer is NULL");

    HMCST_LOG("ENTER: lock=%p, releaser=%p, status_to_pass=%s\n", 
              (void*)lock, (void*)releaser_qnode, st_name(status_to_pass));

    hmcs_node_t* current_node = releaser_qnode;
    bool wrote_impatient_marker = false;
    bool lock_was_passed = false;

    // --- PHASE 1: FORWARD JOURNEY (Recherche de successeur) ---
    assert(current_node != NULL && "hmcst_release_single_level: current_node is NULL at start of forward journey");
    
    while (true) {
        hmcs_node_t* succ = (hmcs_node_t*) vatomicptr_read_acq(&current_node->next);
    HMCST_LOG("FORWARD_INIT: current=%p, succ=%p\n", (void*)current_node, (void*)succ);
         if (succ == HMCST_STATUS_M) {
            HMCST_LOG("FORWARD_PATH: Found M marker. Aborting forward journey.\n");
            goto cleanup_phase;
        }
        if (succ == NULL) {
            // Logique pour gérer la fin de la file et les successeurs lents.
            // Cette partie est identique à hmcst_release_real.
            void* current_tail_value = vatomicptr_read_rlx(&lock->lock);
            void* expected_tail = current_node;
            if (vatomicptr_cmpxchg(&lock->lock, expected_tail, NULL) == expected_tail) {
                HMCST_LOG("FORWARD_PATH: I was the tail. Relinquishing lock. Going to cleanup.\n");
                goto cleanup_phase; // La file est vide, on passe au nettoyage.
            }
            
            cpu_relax();
            assert(current_node != NULL && "hmcst_release_single_level: current_node is NULL after failed CAS");
            succ = (hmcs_node_t*) vatomicptr_read_acq(&current_node->next);
            HMCST_LOG("FORWARD_PATH: Re-read succ after failed CAS. succ=%p\n", (void*)succ);
            if (succ == NULL) {
                void* expected_next_null = NULL;
                if (vatomicptr_cmpxchg(&current_node->next, expected_next_null, (void*)HMCST_STATUS_M) == expected_next_null) {
                    HMCST_LOG("FORWARD_PATH: Wrote M marker. Going to cleanup.\n");
                    wrote_impatient_marker = true;
                    goto cleanup_phase; // On a écrit 'M', on passe au nettoyage.
                }
                 assert(current_node != NULL && "hmcst_release_single_level: current_node is NULL after failed M-write");
                succ = (hmcs_node_t*) vatomicptr_read_acq(&current_node->next);
                 HMCST_LOG("FORWARD_PATH: Re-read succ after failed M-write. succ=%p\n", (void*)succ);
            }
            continue; // On a peut-être un successeur maintenant, on recommence la boucle.
        }

        // On a un successeur. On tente de lui passer le témoin.
        // [MODIFICATION CLÉ] On utilise le paramètre 'status_to_pass'.
        assert(succ != NULL && "hmcst_release_single_level: succ is NULL after check, should be impossible");
        HMCST_LOG("FORWARD_PATH: Found succ=%p. Attempting to pass lock.\n", (void*)succ);
        uint64_t prev_status = vatomic64_xchg(&succ->status, status_to_pass);
        HMCST_LOG("SUCCESSOR CHECK: qnode=%p, prev_status=%s -> %s\n", 
                  (void*)succ, st_name(prev_status), st_name(status_to_pass));

        if (prev_status == HMCST_STATUS_W) {
            // SUCCÈS: Le témoin est passé. La passe avant est terminée.
            HMCST_LOG("FORWARD_PATH: Success, passed lock to waiting successor. Going to cleanup.\n");
            lock_was_passed = true;
                
        }
        
        if (prev_status == HMCST_STATUS_A) {
            // Le successeur a abandonné. On l'impersonnalise.
            // [MODIFICATION CLÉ] On doit remettre le statut 'A' pour que le nœud reste abandonné.
            HMCST_LOG("FORWARD_PATH: Successor %p was abandoned. Impersonating.\n", (void*)succ);
            vatomic64_write_rel(&succ->status, HMCST_STATUS_A);
            assert(succ != NULL && "hmcst_release_single_level: succ is NULL before impersonation");
            hmcs_node_t* next_succ = (hmcs_node_t*) vatomicptr_read_rlx(&succ->next);
            vatomicptr_write_rel(&succ->next, current_node); // Lien arrière
            current_node = succ;
            succ = next_succ;
             HMCST_LOG("FORWARD_PATH: Moving to next pair. new_current=%p, new_succ=%p\n", (void*)current_node, (void*)succ);
            continue; // On continue la recherche avec le successeur suivant.
        }

        // Statut inattendu. On arrête et on nettoie.
        HMCST_LOG("WARNING: Unexpected status %s on successor %p\n", st_name(prev_status), (void*)succ);
        goto cleanup_phase;
    }

cleanup_phase:
    // --- PHASE 2: BACKWARD JOURNEY (Nettoyage) ---
    // Cette partie est identique à hmcst_release_real.

    HMCST_LOG("CLEANUP: Starting backward journey from %p.\n", (void*)current_node);
    
    hmcs_node_t* cleanup_node = current_node;
    while (true) {
        HMCST_LOG("CLEANUP_LOOP: Cleaning node %p.\n", (void*)cleanup_node);

        hmcs_node_t* pred_in_chain = (hmcs_node_t*) vatomicptr_read_rlx(&cleanup_node->next);
        HMCST_LOG("CLEANUP_LOOP: Next predecessor in chain is %p.\n", (void*)pred_in_chain);
        // On ne nettoie pas le nœud si on y a écrit 'M'.
        if (wrote_impatient_marker && cleanup_node == current_node) {
            HMCST_LOG("SKIPPING CLEANUP of M-marked node %p.\n", (void*)cleanup_node);
        } else {
            vatomic64_write_rel(&cleanup_node->status, HMCST_STATUS_R);
        }

        if (cleanup_node == releaser_qnode) {
            HMCST_LOG("CLEANUP_LOOP: Reached original releaser. Cleanup finished.\n");
            break; // On a nettoyé toute la chaîne.
        }
        
        cleanup_node = pred_in_chain;
    }

    HMCST_LOG("COMPLETE: Single-level release finished. Returning %s.\n", lock_was_passed ? "true" : "false");
    return lock_was_passed;

}

void hmcst_n_level_release(hmcslock_t* base_lock, hmcs_node_t* thread_qnode, int n_levels) {
    assert(base_lock != NULL && thread_qnode != NULL && n_levels > 0);
    HMCST_LOG("N-LEVEL RELEASE: ENTER. Releasing %d levels for thread_qnode %p.\n", n_levels, (void*)thread_qnode);

    // --- 1. Préparation : Construire un tableau des verrous de la hiérarchie ---
    hmcslock_t* locks[n_levels + 1];
    hmcslock_t* current_lock_ptr = base_lock;
    for (int i = 1; i <= n_levels; i++) {
        assert(current_lock_ptr != NULL && "Lock hierarchy is shallower than n_levels");
        locks[i] = current_lock_ptr;
        current_lock_ptr = current_lock_ptr->parent;
    }

    // --- 2. Boucle de libération ascendante (bottom-up) ---
    for (int level = 1; level <= n_levels; level++) {
        hmcslock_t* lock_this_level = locks[level];
        hmcs_node_t* releaser_node = (level == 1) ? thread_qnode : &locks[level - 1]->qnode;

        HMCST_LOG("RELEASE (L%d): Analyzing lock %p with releaser_node %p.\n", 
                  level, (void*)lock_this_level, (void*)releaser_node);

        // --- LECTURE ATOMIQUE DE L'ÉTAT ACTUEL ---
        // On lit le statut une seule fois pour garantir la cohérence de la décision.
        uint64_t current_status = vatomic64_read_acq(&releaser_node->status);
        
        // Vérification de sécurité : si le nœud est déjà recyclé, on ne fait rien.
        if (current_status == HMCST_STATUS_R) {
            HMCST_LOG("  (L%d): Releaser_node %p is already Recycled (R). Skipping and ascending.\n", level, (void*)releaser_node);
            continue;
        }

        // --- DÉTERMINATION DE LA VALEUR À PASSER ---
        uint64_t value_to_pass;

        if (level == n_levels) {
            // Cas 1: Au niveau racine, on passe toujours 'U' pour donner le verrou global.
            value_to_pass = HMCST_STATUS_U;
            HMCST_LOG("  (L%d is root): Decision -> Pass U (global lock).\n", level);
        } else {
            // Cas 2: Niveaux intermédiaires, logique du seuil (cohorting).
            // Lecture atomique du seuil pour la comparaison.
            uint32_t threshold = vatomic32_read_acq(&lock_this_level->threshold);
            
            HMCST_LOG("  (L%d non-root): Analyzing status %s against threshold %u.\n", 
                      level, st_name(current_status), threshold);

            if (current_status == HMCST_STATUS_U) {
                // Ce cas est une ERREUR LOGIQUE. Un nœud non-racine ne devrait jamais avoir 'U'.
                // Cela indique un bug dans la fonction d'acquisition.
                HMCST_LOG("  (L%d) CRITICAL ERROR: Non-root releaser_node has U status! Defaulting to pass P.\n", level);
                value_to_pass = HMCST_STATUS_P;
            } else if (current_status < threshold) {
                // Le seuil n'est pas atteint. On incrémente le compteur de passage.
                // Note: C=1, V_START=2. Si status=C, C+1=2=V_START.
                value_to_pass = current_status + 1;
                HMCST_LOG("  (L%d): Status %s < threshold %u. Decision -> Pass incremented value %s.\n", 
                          level, st_name(current_status), threshold, st_name(value_to_pass));
            } else {
                // Le seuil est atteint ou dépassé. On doit passer 'P' pour forcer le successeur à monter.
                value_to_pass = HMCST_STATUS_P;
                HMCST_LOG("  (L%d): Status %s >= threshold %u. Decision -> Pass P (acquire parent).\n", 
                          level, st_name(current_status), threshold, st_name(value_to_pass));
            }
        }

        // --- TENTATIVE DE PASSAGE DU VERROU ---
        if (hmcst_release_single_level(lock_this_level, releaser_node, value_to_pass)) {
            // SUCCÈS ! Le verrou a été passé à un successeur à ce niveau.
            HMCST_LOG("RELEASE (L%d): Successfully passed lock to a successor. Release protocol finished.\n", level);
            
            // La responsabilité du Releaser est terminée.
            return;
        }
        
        // ÉCHEC: Aucun successeur n'a été trouvé à ce niveau. La boucle continue pour monter.
        HMCST_LOG("RELEASE (L%d): No waiting successor found. Ascending to parent level.\n", level);
    }
    
    HMCST_LOG("N-LEVEL RELEASE: COMPLETE. No successor found at any level.\n");
   
     HMCST_LOG("  (Final Cleanup): Starting final cleanup for node %p.\n", (void*)thread_qnode);

    // On va vérifier si on est le dernier détenteur du verrou de base pour le nettoyer.
    // Note: Le code suppose que `base_lock->lock` est le pointeur de queue (tail).
    
    // 1. Lecture atomique de la valeur actuelle du pointeur de queue.
    void* current_tail_value = vatomicptr_read_rlx(&base_lock->lock);
    void* expected_tail = thread_qnode;

    HMCST_LOG("  -> Checking if I am the tail of the base lock. Current tail: %p, My node: %p.\n", 
              current_tail_value, expected_tail);

    // 2. Comparaison : On n'agit que si on est bien le dernier dans la file.
    if (current_tail_value == expected_tail) {
        HMCST_LOG("    --> Condition MET: I am the tail. Attempting to relinquish the lock (CAS tail to NULL).\n");
        
        // 3. Tentative de mise à NULL atomique (Compare-And-Swap).
        void* old_val_before_cas = vatomicptr_cmpxchg(&base_lock->lock, expected_tail, NULL);

        // On vérifie si le CAS a réussi. C'est une bonne pratique pour le logging.
        if (old_val_before_cas == expected_tail) {
            HMCST_LOG("      ---> CAS Successful. Base lock is now free.\n");
        } else {
            // Ce cas est rare mais possible si un autre thread est arrivé entre la lecture et le CAS.
            HMCST_LOG("      ---> CAS FAILED. Race condition detected. Another thread (%p) became the tail.\n", old_val_before_cas);
        }

    } else {
        HMCST_LOG("    --> Condition NOT MET: I am no longer the tail. No need to modify the lock.\n");
    }

    // 4. Nettoyage du nœud du thread lui-même (ceci est toujours fait).
    HMCST_LOG("  -> Recycling my node %p. Setting its 'next' pointer to NULL.\n", (void*)thread_qnode);
    vatomicptr_write_rel(&thread_qnode->next, NULL);
    
    HMCST_LOG("  -> Recycling my node %p. Setting its 'status' to Recyclable (R).\n", (void*)thread_qnode);
    vatomic64_write_rel(&thread_qnode->status, HMCST_STATUS_R);

    HMCST_LOG("  (Final Cleanup): Cleanup complete for node %p.\n", (void*)thread_qnode);
}



*/










pass_result_t hmcst_find_and_pass(hmcslock_t* lock, hmcs_node_t* releaser_qnode, uint64_t status_to_pass) {
    hmcs_node_t* current_impersonator = releaser_qnode;
    hmcs_node_t* next_candidate = (hmcs_node_t*) vatomicptr_read_acq(&current_impersonator->next);
    bool wrote_impatient_marker = false;
    pass_result_t lock_was_passed = PASS_FAILED_EMPTY;

    // --- FORWARD JOURNEY ---
    while (true) {

if (next_candidate == NULL) {
    HMCST_LOG("FIND_PASS: [%p] My 'next' is NULL. Evaluating situation...\n", (void*)releaser_qnode);

    // On lit l'état actuel de la queue pour prendre une décision informée.
    void* current_tail = vatomicptr_read_acq(&lock->lock);

    // CAS 1 : Je suis bien le dernier de la file.
    if (current_tail == current_impersonator) {
        // J'essaie de libérer le verrou en déclarant la file vide.
        HMCST_LOG("FIND_PASS: [%p] I am the tail. Attempting to set tail to NULL.\n", (void*)releaser_qnode);
        
        if (vatomicptr_cmpxchg(&lock->lock, current_impersonator, NULL) == current_impersonator) {
            // SUCCÈS. Le verrou est libre. Mon travail est terminé.
            HMCST_LOG("FIND_PASS: [%p] SUCCESS. Tail is now NULL. Breaking.\n", (void*)releaser_qnode);
            break;
        }
        
        // Le CAS a échoué. Un successeur est arrivé très vite.
        // La chose la plus sûre à faire est de recommencer la boucle pour le traiter.
        HMCST_LOG("FIND_PASS: [%p] CAS on tail failed. A successor was fast. Retrying loop.\n", (void*)releaser_qnode);
        continue;
    }

    // CAS 2: Impatient case detected. Tail has advanced, but my 'next' is still NULL.
            else if (current_tail != NULL) {
                // We DETECT the condition, but we DO NOT act. We report it to the caller.
                HMCST_LOG("FIND_PASS: [%p] IMPATIENT case detected (tail is %p, not me). Reporting to caller.\n",
                          (void*)releaser_qnode, current_tail);
                // The backward cleanup journey is skipped, as we haven't passed any nodes.
            
                return PASS_FAILED_IMPATIENT;
            }
            // ========================================================================
            // CAS 3: Ghost thread case. The lock is already free.
            else { // current_tail == NULL
                HMCST_LOG("FIND_PASS: [%p] Tail is already NULL (ghost thread?). Nothing to do. Breaking.\n", (void*)releaser_qnode);
                break; // Our work is done.
            }
        }

    if (next_candidate == HMCST_STATUS_M) { HMCST_LOG("FIND_PASS: [%p] Found impatient marker (M) in next pointer of node %p. This signifies a handover point. Stopping forward search.\n", 
              (void*)releaser_qnode, (void*)current_impersonator);
              break; }

                    // On a un candidat successeur, 'succ'.
            hmcs_node_t* succ = next_candidate;

            HMCST_LOG("FIND_PASS: [%p] Processing successor %p. Attempting to pass status %s.\n", 
                    (void*)releaser_qnode, (void*)succ, st_name(status_to_pass));

            // L'opération atomique clé : on passe le nouveau statut et on récupère l'ancien.
            uint64_t prev_status = vatomic64_xchg(&succ->status, status_to_pass);

            HMCST_LOG("FIND_PASS: [%p]   - Successor %p's previous status was %s.\n", 
                    (void*)releaser_qnode, (void*)succ, st_name(prev_status));

            // --- Cas 1: Succès, le successeur attendait ---
            if (prev_status == HMCST_STATUS_W) {
                HMCST_LOG("FIND_PASS: [%p] SUCCESS. Found a waiting successor. Lock passed. Breaking forward journey.\n", 
                        (void*)releaser_qnode);
                lock_was_passed = PASS_SUCCESS;
                break; // Succès, on a passé le verrou
            }

                // --- Cas 2: Le successeur avait avorté, on le saute ---
        if (prev_status == HMCST_STATUS_A) {
            HMCST_LOG("FIND_PASS: [%p] Found aborted successor %p. Skipping and adding to cleanup chain.\n", 
                    (void*)releaser_qnode, (void*)succ);
            
            // On lit le pointeur 'next' du nœud avorté AVANT de l'écraser.
            next_candidate = (hmcs_node_t*) vatomicptr_read_rlx(&succ->next);
            
            // On réutilise son champ 'next' pour construire la chaîne de nettoyage vers l'arrière.
            vatomicptr_write_rel(&succ->next, current_impersonator);
            HMCST_LOG("FIND_PASS: [%p]   - Chained %p->next to point back to %p.\n", 
                    (void*)releaser_qnode, (void*)succ, (void*)current_impersonator);
                    
            // On "devient" le nœud avorté pour la suite du parcours.
            current_impersonator = succ;
            HMCST_LOG("FIND_PASS: [%p]   - Now impersonating %p. Continuing search with next candidate %p.\n", 
                    (void*)releaser_qnode, (void*)current_impersonator, (void*)next_candidate);
                    
            continue; // On continue la recherche avec le prochain candidat
        }
        HMCST_LOG("FIND_PASS: [%p] UNEXPECTED STATE. Successor %p had status %s, not W or A. Aborting forward search.\n", 
          (void*)releaser_qnode, (void*)succ, st_name(prev_status));
        break; // Cas inattendu, on arrête
    }
// --- BACKWARD JOURNEY (CLEANUP) ---
HMCST_LOG("FIND_PASS: [%p] Starting backward journey (cleanup) from impersonator %p to original releaser %p.\n",
          (void*)releaser_qnode, (void*)current_impersonator, (void*)releaser_qnode);

hmcs_node_t* cleanup_node = current_impersonator;
while (cleanup_node != releaser_qnode) {
    HMCST_LOG("CLEANUP: [%p]   - Processing intermediate node %p.\n", (void*)releaser_qnode, (void*)cleanup_node);

    // Le champ 'next' pointe maintenant vers le prédécesseur dans la chaîne.
    hmcs_node_t* pred_in_chain = (hmcs_node_t*) vatomicptr_read_rlx(&cleanup_node->next);
    HMCST_LOG("CLEANUP: [%p]     - Predecessor in chain is %p.\n", (void*)releaser_qnode, (void*)pred_in_chain);

    // On ne nettoie que les nœuds intermédiaires (ceux qui ont été "impersonnés").
    HMCST_LOG("CLEANUP: [%p]     - Setting status of intermediate node %p to R (Recyclable).\n",
              (void*)releaser_qnode, (void*)cleanup_node);
    vatomic64_write_rel(&cleanup_node->status, HMCST_STATUS_R);

    cleanup_node = pred_in_chain;
}
HMCST_LOG("FIND_PASS: [%p] Function complete. Lock was passed: %s.\n", 
          (void*)releaser_qnode, lock_was_passed ? "true" : "false");
          
return lock_was_passed;}



static inline bool  hmcst_release_recursive(hmcslock_t* locks[], int current_level, int max_levels, hmcs_node_t* initial_qnode) {

    HMCST_LOG("RELEASE_RECURSIVE: Début pour niveau=%d/%d\n", current_level, max_levels);

    // Condition d'arrêt de la récursion : on a dépassé le niveau racine.
    if (current_level > max_levels) {
        HMCST_LOG("RELEASE_RECURSIVE: Niveau max dépassé, fin de la récursion.\n");
        return;
    }

    // Identification des acteurs pour ce niveau.
    hmcslock_t* lock_this_level = locks[current_level];
    // Le "releaser" est le QNode du thread au niveau 1, ou le QNode imbriqué du verrou parent aux niveaux > 1.
    hmcs_node_t* releaser_node = (current_level == 1) ? initial_qnode : &locks[current_level - 1]->qnode;

    HMCST_LOG("RELEASE_RECURSIVE (L%d): lock=%p, releaser_qnode=%p\n",
              current_level, (void*)lock_this_level, (void*)releaser_node);

    // Lecture du statut 'V' du releaser pour ce niveau.
    uint64_t current_status = vatomic64_read_acq(&releaser_node->status);

    // Cas spécial pour le niveau racine (le plus haut niveau).
    // Il n'a pas de parent à libérer. Il passe simplement le verrou avec 'U' (Unlocked) et c'est fini.
    if (current_level == max_levels) {
        HMCST_LOG("RELEASE_RECURSIVE (L%d): Niveau racine. Passage avec 'U'.\n", current_level);
        hmcst_find_and_pass(lock_this_level, releaser_node, HMCST_STATUS_U);
        vatomic64_write_rel(&releaser_node->status, HMCST_STATUS_R); // Marquer comme recyclable.
        return;
    }

    // --- LOGIQUE DE LIBÉRATION UNIFIÉE ---

    // On lit le seuil de passage local (theta).
    uint32_t threshold = vatomic32_read_acq(&lock_this_level->threshold);

    // 1. TENTATIVE DU CHEMIN RAPIDE (si sous le seuil)
    if (current_status < threshold) {
        uint64_t value_to_pass = current_status + 1;
        HMCST_LOG("RELEASE_RECURSIVE (L%d): Statut (%llu) < Seuil (%u). Tentative de passage local avec %s.\n",
                  current_level, (unsigned long long)current_status, threshold, st_name(value_to_pass));

        pass_result_t result = hmcst_find_and_pass(lock_this_level, releaser_node, value_to_pass);

        switch (result) {
            case PASS_SUCCESS:
                // SUCCÈS ! Le verrou a été passé à un voisin. Le travail est entièrement terminé.
                HMCST_LOG("RELEASE_RECURSIVE (L%d): Succès du passage local. La libération est terminée.\n", current_level);
                vatomic64_write_rel(&releaser_node->status, HMCST_STATUS_R);
                return; // C'est la seule condition de sortie anticipée.
            
            case PASS_FAILED_IMPATIENT:
                // CAS IMPATIENT DÉTECTÉ. C'est ici qu'on applique le protocole spécial.
                HMCST_LOG("RELEASE_RECURSIVE (L%d): Cas impatient détecté. Exécution du protocole spécial.\n", current_level);
                
                // Étape I: D'abord, libérer le parent.
                HMCST_LOG("RELEASE_RECURSIVE (L%d):   Étape I - Libération du parent (appel récursif pour L%d).\n", current_level, current_level + 1);
                hmcst_release_recursive(locks, current_level + 1, max_levels, initial_qnode);

                // Étape II: Ensuite, écrire le marqueur 'M'.
                HMCST_LOG("RELEASE_RECURSIVE (L%d):   Étape II - Parent libéré. Écriture du marqueur 'M'.\n", current_level);
                void* expected_next_null = NULL;
                void* observed_next = vatomicptr_cmpxchg(&releaser_node->next, expected_next_null, (void*)HMCST_STATUS_M);

                // On vérifie si le CAS a réussi.
                if (observed_next == expected_next_null) {
                    // SUCCÈS : Le CAS a fonctionné. Notre pointeur 'next' était bien NULL.
                    // Le marqueur 'M' est maintenant en place. Notre travail est terminé.
                    HMCST_LOG("RELEASE_RECURSIVE (L%d):     [SUCCÈS DU CAS] Le marqueur 'M' a été écrit avec succès dans le 'next' de QNode %p.\n", 
              current_level, (void*)releaser_node);   
              return true;             }
                        
               else {
                    // ÉCHEC : Le CAS a échoué. Cela signifie qu'un successeur a réussi à se lier
                    // pendant que nous libérions le parent. La situation a changé.
                    HMCST_LOG("RELEASE_RECURSIVE (L%d):     [ÉCHEC DU CAS] Le pointeur 'next' n'était plus NULL (valeur observée: %p). Un successeur s'est manifesté.\n", current_level, observed_next);
                    
                    // Conformément au protocole, puisque le parent est libéré et qu'un successeur
                    // est présent, nous devons lui passer le drapeau 'P'.
                    HMCST_LOG("RELEASE_RECURSIVE (L%d):     Transition vers le passage local avec 'P' pour notifier le successeur.\n", current_level);
                    hmcst_find_and_pass(lock_this_level, releaser_node, HMCST_STATUS_P);
                }
                // Le travail pour cette branche de la libération est terminé.
                vatomic64_write_rel(&releaser_node->status, HMCST_STATUS_R);
                return; // IMPORTANT: On sort ici, on ne va PAS au chemin de secours.

            case PASS_FAILED_EMPTY:
                // ÉCHEC du passage local (pas de successeur). On laisse l'exécution "tomber" (fall-through)
                // dans le chemin de secours ci-dessous.
                HMCST_LOG("RELEASE_RECURSIVE (L%d): Échec du passage local (pas de successeur). Passage au protocole de secours.\n", current_level);
                break; // On sort du switch pour continuer vers le chemin de secours.
        }
    }

    // 2. EXÉCUTION DU CHEMIN DE SECOURS (Fallback)
    // Ce code est atteint si :
    //   - Le statut était >= au seuil.
    //   - OU le statut était < au seuil, mais le passage local a échoué.
    // C'est la logique correcte qui couvre tous les cas requis par le papier.
    HMCST_LOG("RELEASE_RECURSIVE (L%d): Exécution du protocole de secours.\n", current_level);

    // Étape I: D'abord, libérer le parent.
    HMCST_LOG("RELEASE_RECURSIVE (L%d):   Étape I - Libération du parent (appel récursif pour L%d).\n", current_level, current_level + 1);
    hmcst_release_recursive(locks, current_level + 1, max_levels, initial_qnode);

    // Étape II: Ensuite, notifier un successeur local avec 'P'.
    HMCST_LOG("RELEASE_RECURSIVE (L%d):   Étape II - Parent libéré. Notification locale avec 'P'.\n", current_level);
    hmcst_find_and_pass(lock_this_level, releaser_node, HMCST_STATUS_P);
    
    // Le travail pour ce niveau est terminé, on marque le QNode comme recyclable.
    vatomic64_write_rel(&releaser_node->status, HMCST_STATUS_R);
    HMCST_LOG("RELEASE_RECURSIVE (L%d): Fin du travail pour ce niveau.\n", current_level);
}


static inline
void hmcst_release(hmcslock_t* base_lock, hmcs_node_t* thread_qnode, int n_levels) {

assert(base_lock != NULL && thread_qnode != NULL && n_levels > 0);

HMCST_LOG("HMCST_RELEASE: ENTER. Releasing %d levels for thread_qnode %p.\n",

n_levels, (void*)thread_qnode);


// Préparation du tableau des verrous

hmcslock_t* locks[n_levels + 1];

hmcslock_t* current_lock_ptr = base_lock;

for (int i = 1; i <= n_levels; i++) {

assert(current_lock_ptr != NULL && "Lock hierarchy is shallower than n_levels");

locks[i] = current_lock_ptr;

current_lock_ptr = current_lock_ptr->parent;

}


// Lancer la récursion

 bool cleanup_was_delegated = hmcst_release_recursive(locks, 1, n_levels, thread_qnode);

    // On ne nettoie le QNode du thread que si la responsabilité n'a pas été déléguée.
    if (!cleanup_was_delegated) {
        vatomic64_write_rel(&thread_qnode->status, HMCST_STATUS_R);
    } else {
        HMCST_LOG("HMCST_RELEASE: Responsibility was delegated via M. Skipping cleanup of thread's QNode %p.\n", (void*)thread_qnode);
    }
HMCST_LOG("HMCST_RELEASE: COMPLETE\n");

}





/**
 * @brief Nettoie un segment de la file d'attente en marquant les noeuds 'A' en 'R'.
 * 
 * Implémente une technique de nettoyage non-bloquante en deux passes ("Fil d'Ariane").
 * 1. Passe Avant: Parcourt le segment, inversant les pointeurs 'next' des noeuds 'A'
 *    pour créer un chemin de retour.
 * 2. Passe Arrière: Suit ce chemin de retour pour marquer les noeuds 'A' comme 'R'.
 * 
 * @param start_node Le premier noeud du segment à nettoyer (le noeud de 'α').
 * @param stop_at_succ Le successeur qui délimite la fin du segment. Le parcours s'arrête
 *                     juste avant d'atteindre ce noeud.
 */
void hmcst_cleanup_path(hmcs_node_t* start_node, hmcs_node_t* stop_at_succ) {

    HMCST_LOG("CLEANUP_PATH: Début du nettoyage depuis %p jusqu'à avant %p.\n", 
              (void*)start_node, (void*)stop_at_succ);

    // 'current_impersonator' est le dernier noeud 'A' que nous avons "inversé".
    // Au début, c'est notre propre noeud.
    hmcs_node_t* current_impersonator = start_node;
    
    // ========================================================================
    // PASSE 1 : PARCOURS AVANT (Déposer le Fil d'Ariane)
    // ========================================================================
    
    // On commence par inspecter le successeur de notre noeud de départ.
    hmcs_node_t* next_candidate = (hmcs_node_t*) vatomicptr_read_acq(&current_impersonator->next);

    while (next_candidate != NULL && next_candidate != stop_at_succ) {
        
        hmcs_node_t* succ = next_candidate;
        
        // On ne s'intéresse qu'aux noeuds abandonnés.
        if (vatomic64_read_acq(&succ->status) == HMCST_STATUS_A) {
            
            // Étape 1: Sauvegarder le chemin vers l'avant AVANT de le modifier.
            next_candidate = (hmcs_node_t*) vatomicptr_read_rlx(&succ->next);
            
            // Étape 2: Inverser le pointeur 'next' pour qu'il pointe vers l'arrière.
            // C'est notre "fil d'Ariane".
            vatomicptr_write_rel(&succ->next, current_impersonator);
            
            // Étape 3: Avancer notre "impersonnation". Nous sommes maintenant ce noeud 'A'.
            current_impersonator = succ;
            
            // On continue la boucle avec le chemin sauvegardé.
            continue;
        }

        // Si le successeur n'est pas 'A', on ne le touche pas.
        // On avance simplement notre point de vue pour continuer le scan.
        current_impersonator = succ;
        next_candidate = (hmcs_node_t*) vatomicptr_read_acq(&current_impersonator->next);
    }

    // ========================================================================
    // PASSE 2 : PARCOURS ARRIÈRE (Suivre le Fil et Nettoyer)
    // ========================================================================
    
    HMCST_LOG("CLEANUP_PATH: Fin du parcours avant. Début du nettoyage arrière depuis %p.\n", 
              (void*)current_impersonator);

    hmcs_node_t* cleanup_node = current_impersonator;
    
    // On remonte la chaîne jusqu'à notre point de départ.
    while (cleanup_node != start_node) {
        
        // Suivre le "fil d'Ariane" pour trouver le prédécesseur dans notre chaîne.
        hmcs_node_t* pred_in_chain = (hmcs_node_t*) vatomicptr_read_rlx(&cleanup_node->next);
        
        // On ne nettoie que les noeuds qui sont bien des 'A'.
        // C'est une sécurité supplémentaire.
        if (vatomic64_read_acq(&cleanup_node->status) == HMCST_STATUS_A) {
             vatomic64_write_rel(&cleanup_node->status, HMCST_STATUS_R);
             HMCST_LOG("CLEANUP_PATH:   - Marqué le noeud %p comme 'R'.\n", (void*)cleanup_node);
        }
       
        // Avancer vers l'arrière.
        cleanup_node = pred_in_chain;
    }
    
    HMCST_LOG("CLEANUP_PATH: Nettoyage terminé.\n");
}



/**
 * @brief Scanne une file à la recherche d'un successeur 'W' pour le passage global.
 * @param start_node Le noeud de l'aborteur à ce niveau.
 * @param out_found_successor [OUT] Adresse du successeur 'W' trouvé et marqué 'P'. 
 * @param out_last_scanned_node [OUT] Adresse du dernier noeud inspecté ('m').
 */
static inline find_peer_result_t
hmcst_scan_for_successor(hmcs_node_t* start_node, 
                         hmcs_node_t** out_found_successor, 
                         hmcs_node_t** out_last_scanned_node) 
{
    // 'current_node' est notre point de vue courant. C'est lui qui deviendra 'm'.
    hmcs_node_t* current_node = start_node;

    while (true) {
        hmcs_node_t* next_candidate = (hmcs_node_t*) vatomicptr_read_acq(&current_node->next);

        // Cas 1: Fin de la chaîne.
        if (next_candidate == NULL) {
            HMCST_LOG("SCAN_SUCC: [%p] Reached end of chain.\n", (void*)start_node);
            *out_found_successor = NULL;
            *out_last_scanned_node = current_node; // 'm' est le dernier noeud qu'on a tenu.
            return PEER_NOT_FOUND;
        }

        // On ignore les marqueurs 'M' dans cette phase de recherche pure.
        if (next_candidate == HMCST_STATUS_M) {
            HMCST_LOG("SCAN_SUCC: [%p] Found impatient marker (M). End of search.\n", (void*)start_node);
            *out_found_successor = NULL;
            *out_last_scanned_node = current_node;
            return PEER_NOT_FOUND;
        }

        hmcs_node_t* succ = next_candidate;
        
        // Utilisons un CAS pour la robustesse. On lit et on tente de modifier en une seule fois.
        uint64_t succ_status = vatomic64_read_acq(&succ->status);

        // Cas 2: Succès ! Le successeur attendait.
        if (succ_status == HMCST_STATUS_W) {
            // On tente de le "réclamer" en passant son statut à 'P'.
            if (vatomic64_cmpxchg(&succ->status, HMCST_STATUS_W, HMCST_STATUS_P) == HMCST_STATUS_W) {
                HMCST_LOG("SCAN_SUCC: [%p] SUCCESS. Found and tagged waiting successor %p with 'P'.\n", (void*)start_node, (void*)succ);
                *out_found_successor = succ;
                *out_last_scanned_node = current_node; // Le noeud qui le précède.
                return PEER_FOUND;
            }
            // Si le CAS a échoué, son statut a changé. On recommence la boucle pour le ré-évaluer.
            continue;
        }

        // Cas 3: Le successeur a lui-même abandonné. On le saute CORRECTEMENT.
        if (succ_status == HMCST_STATUS_A) {
            HMCST_LOG("SCAN_SUCC: [%p] Found aborted successor %p. Skipping it.\n", (void*)start_node, (void*)succ);
            // On avance notre point de vue pour continuer la recherche à partir de lui.
            current_node = succ;
            continue;
        }

        // Cas 4: État inattendu. La chaîne est rompue.
        HMCST_LOG("SCAN_SUCC: [%p] UNEXPECTED STATE. Successor %p had status %s. Aborting scan.\n", (void*)start_node, (void*)succ, st_name(succ_status));
        *out_found_successor = NULL;
        *out_last_scanned_node = current_node;
        return PEER_NOT_FOUND;
    }
}

void hmcst_abort(hmcslock_t* base_lock, int n_levels, hmcs_node_t* qnode, int abort_level) {

    
    vatomic64_write_rel(&qnode->status, HMCST_STATUS_A);

    // --- PRÉ-CONDITION ---
    if (abort_level <= 1) {
        HMCST_LOG("ABORT_END: Avortement au niveau <= 1. Aucune action complexe requise. Fin.\n");
        return;
    }

hmcslock_t* locks[n_levels];
    hmcs_node_t* my_qnodes[n_levels];

    hmcslock_t* current_lock_ptr = base_lock;
    for (int i = 0; i < n_levels; i++) {
        if (current_lock_ptr == NULL) {
            HMCST_LOG("ABORT_ERROR: Hiérarchie de verrous invalide.\n");
            return;
        }
        
        // Remplir le tableau des verrous
        locks[i] = current_lock_ptr;
        
        // Remplir le tableau des QNodes
        if (i == 0) {
            // Pour le niveau 1 (i=0), c'est le QNode personnel du thread
            my_qnodes[i] = qnode;
        } else {
            // Pour les niveaux > 1, c'est le QNode imbriqué dans le verrou du niveau précédent
            my_qnodes[i] = &locks[i - 1]->qnode;
        }
        
        HMCST_LOG("ABORT_INIT: L%d -> lock %p, qnode %p\n", i + 1, (void*)locks[i], (void*)my_qnodes[i]);
        
        // Passer au parent pour la prochaine itération
        current_lock_ptr = current_lock_ptr->parent;
    }
    HMCST_LOG("ABORT_INIT: Reconstruction terminée.\n");

    HMCST_LOG("ABORT_START: Thread %p avorte au niveau %d. Début du protocole.\n",
              (void*)my_qnodes[0], abort_level);



    // ========================================================================
    // ÉTAPES 2 & 3: Recherche Ascendante du Successeur Global
    // ========================================================================
    hmcs_node_t* successor_s = NULL;
    int successor_level = -1;
    hmcs_node_t* rightmost_peers[n_levels]; 
    memset(rightmost_peers, 0, sizeof(hmcs_node_t*) * n_levels);
    HMCST_LOG("ABORT_ASCENT: Début de la recherche ascendante [L1...L%d] pour un héritier global.\n", abort_level - 1);

    for (int curLevel = 1; curLevel < abort_level; curLevel++) {
        hmcs_node_t* my_qnode_at_level = my_qnodes[curLevel - 1];
        HMCST_LOG("ABORT_ASCENT (L%d): Scan depuis mon QNode %p...\n", curLevel, (void*)my_qnode_at_level);

        find_peer_result_t result = hmcst_scan_for_successor(
            my_qnode_at_level, 
            &successor_s,
            &rightmost_peers[curLevel - 1]
        );
        
        if (result == PEER_FOUND) {
            successor_level = curLevel;
            HMCST_LOG("ABORT_ASCENT: SUCCÈS! Trouvé héritier global %p au niveau %d. Arrêt de la montée.\n", 
                      (void*)successor_s, successor_level);
            break; 
        }
        
        HMCST_LOG("ABORT_ASCENT (L%d): Aucun héritier trouvé. Montée au niveau suivant.\n", curLevel);
    }

    // ========================================================================
    // ÉTAPE 4: Nettoyage ou Abandon Global
    // ========================================================================
    hmcs_node_t* impatient_node_from_relinquish = NULL;

    if (successor_s != NULL) {
        // CAS 1: Successeur trouvé
        HMCST_LOG("ABORT_CLEANUP (L%d): Héritier trouvé. Nettoyage du chemin de %p jusqu'à avant %p.\n",
                  successor_level, (void*)my_qnodes[successor_level - 1], (void*)successor_s);
        hmcst_cleanup_path(my_qnodes[successor_level - 1], successor_s);
    } else {
        // CAS 2: Aucun successeur global trouvé
        HMCST_LOG("ABORT_RELINQUISH: Aucun héritier global trouvé. Tentative d'abandon du plus haut verrou.\n");
        int top_level_held = abort_level - 1;

        if (top_level_held > 0) {
            HMCST_LOG("ABORT_RELINQUISH (L%d): Appel de find_and_pass pour abandonner le verrou.\n", top_level_held);
            
            pass_result_t result = hmcst_find_and_pass(
                locks[top_level_held - 1],
                my_qnodes[top_level_held - 1],
                HMCST_STATUS_P
            );

            if (result == PASS_FAILED_IMPATIENT) {
                HMCST_LOG("ABORT_RELINQUISH (L%d): CAS IMPATIENT DÉTECTÉ! Mémorisation du noeud %p pour plus tard.\n",
                          top_level_held, (void*)my_qnodes[top_level_held - 1]);
                impatient_node_from_relinquish = my_qnodes[top_level_held - 1];
            } else {
                HMCST_LOG("ABORT_RELINQUISH (L%d): Abandon du verrou terminé sans cas impatient.\n", top_level_held);
            }
        }
        
        HMCST_LOG("ABORT_RELINQUISH: Préparation pour la descente complète.\n");
        successor_level = abort_level;
    }

    // ========================================================================
    // ÉTAPE 5: Descente et Libération Locale
    // ========================================================================
    int descent_start_level = successor_level - 1;
    HMCST_LOG("ABORT_DESCENT: Début de la descente de L%d à L1 pour libération locale.\n", descent_start_level);
    
    hmcs_node_t* impatient_node_from_descent = NULL;

    for (int curLevel = descent_start_level; curLevel >= 1; curLevel--) {
        HMCST_LOG("ABORT_DESCENT (L%d): Traitement de la libération locale.\n", curLevel);

        hmcs_node_t* start_node_for_pass = rightmost_peers[curLevel - 1];
        
        if (start_node_for_pass == NULL) {
            start_node_for_pass = my_qnodes[curLevel - 1];
            HMCST_LOG("ABORT_DESCENT (L%d): Pas de 'm' sauvegardé, départ de mon propre QNode %p.\n",
                      curLevel, (void*)start_node_for_pass);
        } else {
            HMCST_LOG("ABORT_DESCENT (L%d): Reprise de la recherche depuis le noeud 'm' sauvegardé: %p.\n",
                      curLevel, (void*)start_node_for_pass);
        }
       
        HMCST_LOG("ABORT_DESCENT (L%d): Appel de find_and_pass...\n", curLevel);
        pass_result_t result = hmcst_find_and_pass(
            locks[curLevel - 1], 
            start_node_for_pass, 
            HMCST_STATUS_P
        );

        if (result == PASS_FAILED_IMPATIENT && impatient_node_from_descent == NULL) {
            HMCST_LOG("ABORT_DESCENT (L%d): CAS IMPATIENT DÉTECTÉ! Mémorisation du noeud %p. C'est le premier cas rencontré en descente.\n",
                      curLevel, (void*)start_node_for_pass);
            impatient_node_from_descent = start_node_for_pass;
        }
    }
    HMCST_LOG("ABORT_DESCENT: Fin de la boucle de descente.\n");

    // ========================================================================
    // PHASE POST-DESCENTE: Gestion Finale du Cas Impatient
    // ========================================================================
    HMCST_LOG("ABORT_POST_DESCENT: Vérification s'il y a un cas impatient à traiter.\n");

    hmcs_node_t* node_to_mark_impatient = NULL;
    if (impatient_node_from_descent != NULL) {
        node_to_mark_impatient = impatient_node_from_descent;
        HMCST_LOG("ABORT_POST_DESCENT: Cas impatient trouvé pendant la descente. Noeud à marquer: %p.\n", (void*)node_to_mark_impatient);
    } else if (impatient_node_from_relinquish != NULL) {
        node_to_mark_impatient = impatient_node_from_relinquish;
        HMCST_LOG("ABORT_POST_DESCENT: Cas impatient trouvé pendant l'abandon global. Noeud à marquer: %p.\n", (void*)node_to_mark_impatient);
    } else {
        HMCST_LOG("ABORT_POST_DESCENT: Aucun cas impatient n'a été détecté. Rien à faire.\n");
    }

    if (node_to_mark_impatient != NULL) {
        HMCST_LOG("ABORT_POST_DESCENT: Action! Tentative d'écriture du marqueur 'M' dans le 'next' de %p.\n", (void*)node_to_mark_impatient);
        
        void* expected_next_null = NULL;
        void* observed_next = vatomicptr_cmpxchg(&node_to_mark_impatient->next, expected_next_null, (void*)HMCST_STATUS_M);

        if (observed_next == expected_next_null) {
            HMCST_LOG("ABORT_POST_DESCENT: SUCCÈS du CAS. Le marqueur 'M' a été écrit.\n");
        } else {
            HMCST_LOG("ABORT_POST_DESCENT: ÉCHEC du CAS. Le 'next' n'était plus NULL (valeur: %p). Un successeur a été plus rapide. Aucune autre action n'est prise.\n", observed_next);
        }
    }

    HMCST_LOG("ABORT_END: Protocole d'abandon terminé pour le thread %p.\n", (void*)my_qnodes[0]);
}