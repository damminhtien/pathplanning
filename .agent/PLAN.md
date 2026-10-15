# Plan: Performance audit and root-cause fixes

| Milestone | Verification boundary | Status |
| --- | --- | --- |
| M1. Profile representative continuous workloads | Record native/API timings and counters for PRM* and RIT* on fixed BARN cases. | done |
| M2. Fix static native collision checking | Exact segment checks cover boxes, spheres, and OBBs; sparse-step collision regression passes. | done |
| M3. Reduce RIT* hot-loop overhead | Reuse per-plan metric workspaces and bound/index the 2D CARM Gaussian tail; preserve deterministic path cost. | done |
| M4. Verify and checkpoint | Run focused tests, native build, lint/type checks, repeat targeted profiles, refresh Graphify, inspect and commit. | done |
