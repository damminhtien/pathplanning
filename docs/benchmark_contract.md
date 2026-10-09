# Benchmark Report Contract

The two repository benchmark commands use the versioned
`pathplanning_benchmark_v1` JSON contract:

- `python scripts/benchmark_planners.py`
- `python scripts/benchmark_native_sampling.py`

Pass `--output path/to/report.json` to write a report atomically. `--json`
prints the same report to stdout. The output path is excluded from the source
fingerprint so saving a report does not change the experiment identity.

This v1 contract covers those two general repository commands. The dataset
campaigns use source-aware runners and distinct contracts for grid,
directed-road, voxel, and geometric inputs; see the
[shortest-path benchmark contract](shortest_path_benchmark_contract.md) and
[dataset characterization](benchmarks/dataset_characterization.md). Do not
interpret v1 reports as the combined dataset-campaign record.

## Identity and provenance

Each report records:

- `schema_version`, `benchmark_id`, a unique `run_id`, and a stable
  `experiment_id`;
- exact workload and setting descriptions, including measured and warm-up
  seed rules;
- UTC start and finish times, duration, and command line;
- platform, processor, Python and NumPy versions, configured C/C++ compiler
  details, and build flags;
- Git commit, branch, dirty state, a worktree fingerprint, and hashes for
  untracked files;
- paths, byte sizes, and SHA-256 digests for native search libraries found in
  the checkout.

The experiment ID hashes the benchmark name, settings, workloads, source
provenance, host environment, and native artifact digests. It deliberately
excludes run ID, timestamps, output location, and measured values. A dirty
worktree is therefore identifiable; the report does not imply it came from a
clean commit. Graphify's generated `graphify-out/` files,
`benchmark-results/`, and the requested output path are excluded from the
source fingerprint.

## Raw observations and summaries

Every invocation is stored in `runs`. A row includes its case and variant,
`phase` (`warmup` or `measure`), repeat index, seed, timestamps, status,
elapsed time, measurements, and outcomes. Failed invocations remain visible
with an error type and message. Warm-ups are retained but never included in
`results`.

Each `results` row groups one case and variant. `attempt_count`,
`completed_count`, and `error_count` describe measured invocations.
`success_rate` is `success_count / completed_count`; its denominator is
named in `success_rate_denominator`. A planner that completed but found no
path counts as unsuccessful. Each numeric measurement reports its non-null
sample count, median, minimum, and maximum. Metric counts can be lower than
`completed_count` when a measurement is legitimately absent, such as path
cost when no path was found.

## Reproducibility and interpretation

Measured repetitions use `seed + repeat_index`. The native sampling benchmark
pairs both space paths on the same seed and alternates their execution order.
Warm-ups use separate seeds after the measured seed range. These rules make
the input sequence reproducible; wall-clock timing can still vary with host
load, thermal state, operating system scheduling, and compiler/runtime builds.

The representative planner benchmark covers the exact three workloads listed
in each report. The native sampling benchmark covers its listed planners,
scenario, and two space paths. Results characterize those workloads only.
`examples/bench/nn_index_bench.py` is an instructional microbenchmark and
does not emit this contract, so it is not canonical performance evidence.

The 2026-10-02 native sampling report predates this contract. Its recorded
aggregate values remain historical, summary-only evidence; the missing
run-level observations and source fingerprint cannot be reconstructed from
that report. New benchmark runs emit v1 reports.
