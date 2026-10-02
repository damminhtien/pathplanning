# Native Sampling Benchmark — 2026-10-02

This report compares each registered sampling planner using a callback-free
native space model and an equivalent `Grid2DSamplingSpace` subclass that routes
space operations through Python callbacks. Both variants run through
`plan_continuous`; the report separates end-to-end API time from the C engine's
timer.

This report predates the shared v1 benchmark contract. It preserves aggregate
results only; it does not include the original per-run observations or source
fingerprint. Do not treat it as a v1 report. New runs use the schema documented
in [the benchmark contract](../benchmark_contract.md).

## Setup

- Host: macOS 26.7.1, Intel Core i7-1068NG7, 8 logical CPUs
- Python: 3.12.14; NumPy: 2.5.3
- C/C++ compiler: Homebrew clang 23.1.0
- Build flags: C11/C++17 with `-O3`
- Scenario: 10 × 10 2D bounds, one 2 × 2 rectangle obstacle, start `(1, 1)`,
  goal `(9, 9)`, step size `0.5`, collision step `0.25`
- Planner settings: 1,000 maximum iterations, 256 samples, radius gamma 8.0
- Runs: five seeds (7–11) per planner and mode, plus one warm-up
- Both modes solved all five runs. Full metadata, success, node counts, and path
  costs are in [`native_sampling_2026-10-02.json`](native_sampling_2026-10-02.json).

## Results

Median milliseconds. The ratio compares the callback-backed full API median to
the callback-free native-model median for the same planner.

| Planner | Native model API | Python callback API | API ratio | Native model C | Python callback C |
| --- | ---: | ---: | ---: | ---: | ---: |
| `rrt` | 0.413 | 13.095 | 31.7× | 0.115 | 12.937 |
| `rrt_star` | 3.527 | 325.306 | 92.2× | 3.236 | 325.092 |
| `informed_rrt_star` | 4.561 | 535.001 | 117.3× | 4.166 | 534.706 |
| `fmt_star` | 1.150 | 21.258 | 18.5× | 0.528 | 20.875 |
| `bit_star` | 1.149 | 43.632 | 38.0× | 0.819 | 43.303 |
| `abit_star` | 0.667 | 4.966 | 7.5× | 0.303 | 4.787 |
| `rrt_connect` | 0.304 | 8.340 | 27.4× | 0.046 | 8.142 |

In this workload, removing per-expansion Python space callbacks reduced median
full-API time by 7.5×–117.3×. Most of the difference appears inside the C timer,
which includes time spent waiting for callbacks. This measures callback cost
within the current C planner engine; it is not a comparison against the former
Python planner implementation or a guarantee for other spaces and workloads.
