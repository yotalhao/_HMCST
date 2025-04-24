# _HMCST
Based on the paper: “An Efficient Abortable-locking Protocol for Multi-level NUMA Systems”

---

# HMCS-T Lock Implementation in C

This project is a personal academic endeavor by **Youssef Talhaoui**, developed as part of a Master's thesis. It provides a full implementation of the **HMCS-T (Hierarchical MCS with Timeout and Abandonment)** locking protocol in C, extending the classic HMCS locking mechanism with support for **timeouts**, **abandonment**, and **robust multi-level queue management**.

## Overview

HMCS-T is a scalable, hierarchical spinlock designed for modern multi-core and NUMA systems. It addresses the limitations of standard HMCS locks by adding support for:

- **Timeouts**: Threads can give up waiting after a certain time.
- **Abandonment**: Threads that timeout are properly handled to avoid blocking successors.
- **Cohort-awareness**: Locks propagate ownership hierarchically, reducing contention.

This implementation includes:
- A custom header `vsync/hmcstlock.h` defining the protocol logic.
- Debug macros for fine-grained logging (`HMCST_DBG`).
- Full cycle-accurate timestamping using TSC (`rdtsc`).
- Robust handling of cycles and abandoned nodes in the MCS queue.

## Purpose

This implementation is intended for **educational purposes**, performance benchmarking, and comparison against standard HMCS locks. It is not yet production-ready but aims to explore the practicality and complexity of real-world adaptive locks in C.

## Author

**Youssef Talhaoui**  
Université Libre de Bruxelles (ULB)  
Master's Degree in Computer Science – Computational Intelligence & Data Science  

---

Let me know if you want to add sections for benchmark results, usage examples, or contribution guidelines!
