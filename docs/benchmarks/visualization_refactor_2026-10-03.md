# Visualization refactor: performance and validation

## Production boundary

The normal search and continuous libraries are built without `PP_ENABLE_TRACE`.
The diagnostic variants use the same `.cpp` and `.c` sources with that macro
enabled. Their object files are isolated by extension name during builds. Symbol
inspection confirmed there are no trace exports in the production binaries;
the CSR importer and graph-storage counter exist only in the diagnostic search
library. Matplotlib stays unloaded during ordinary planning.

The production comparison used commit `b4da348` as the before snapshot and the
working tree after the visualization refactor as the after version. Both used
the same seed sequence, 15 measured runs, 2 warmups, and fixed limits. The
production/diagnostic comparison used the same limits and 15 paired seeds.
Every measured pair returned matching success, stop reason, iteration and node
counts, path, and path cost. No trace run was truncated.

## Production before and after

| Workload | Full API median before → after | Native stage median before → after | Paired median change in native stage | Peak RSS median before → after |
| --- | ---: | ---: | ---: | ---: |
| A* on 81×81 grid | 10.60 → 11.81 ms | 0.195 → 0.203 ms | +2.5% | 49.57 → 49.62 MiB |
| RRT* on 10×10 space, 1,000 iterations | 18.73 → 21.28 ms | 7.275 → 8.323 ms | −5.6% | 49.63 → 49.71 MiB |

The measurements are noisy at these short runtimes. Production full-API
coefficient of variation was 29–35% for A* and 21–22% for RRT*. Native-stage
variation was 12–15% for A* and 30–36% for RRT*. Paired RRT* native-stage
changes ranged from −49.6% to +78.2%. The small median differences do not
establish a performance regression or gain. Peak process RSS changed by less
than 0.2%; the production benchmark has zero trace payload and never loads the
diagnostic libraries.

The production code path is protected at compile time: trace recorders and
calls are excluded by preprocessing, and the built production binaries expose
no trace symbols. The timing campaign is consistent with that isolation but is
not precise enough to claim zero runtime variation across builds.

## Diagnostic tracing cost

| Workload | Full API median production → diagnostic | Native stage median production → diagnostic | Diagnostic trace payload | Diagnostic graph copy |
| --- | ---: | ---: | ---: | ---: |
| A* | 11.50 → 18.02 ms | 0.202 → 1.046 ms | 60,576 B, 1,893 events | 860,336 B |
| RRT* | 9.49 → 10.58 ms | 4.396 → 5.314 ms | 138,608 B, 3,374 events | — |

These values describe the diagnostic build only. The discrete native-stage
timer includes cloning the production graph from a borrowed CSR view and then
searching it. `graph_bytes` includes the diagnostic graph object and allocated
CSR-vector capacities; it is separate from `TraceOptions.max_bytes`. Peak RSS
includes Python and imported libraries, so it does not isolate trace allocation.
Timing variation in this campaign was also high, especially for the RRT*
workload; use the per-run values and dispersion in the raw report when
interpreting these medians.

## Workloads and environment

- A*: 81×81 grid, one wall with a six-cell gap, start `(5, 5)`, goal `(75, 75)`,
  maximum 20,000 expansions.
- RRT*: 10×10 grid with one 2×2 rectangle, start `(1, 1)`, goal `(9, 9)`,
  1,000 iterations, step size `0.5`.
- Host: macOS 26.7.1, x86_64, Python 3.12.14, NumPy 2.5.3, Homebrew Clang
  23.1.0; native builds used `-O3`, C11, and C++17.
- Each observation ran in a fresh process. Peak RSS is reported in bytes in the
  JSON files.

## Validation

- Full suite: 154 passed.
- Ruff lint and formatting checks passed.
- Production and diagnostic C/C++ sources compiled with `-Wall -Wextra -Werror`.
- Headless rendering covered independent figures, 2D/3D scenes, failed plans,
  bidirectional and anytime replay, reverse seeking, truncation, Dynamic RRT
  pruning/regrowth, and viewer cleanup.
- Pyright has three existing diagnostics in unchanged `core/contracts.py`; it
  reported no diagnostics in the new trace or visualization files.

Raw per-run data, environment metadata, library hashes, and seed-paired
comparisons are available in the [production comparison](visualization_production_2026-10-03.json)
and [diagnostic overhead report](visualization_trace_2026-10-03.json).
