# Reproducing the single-agent shortest-path benchmark

This runner implements the MovingAI `land_octile_v1` profile from the
[Moving AI 2D benchmark collection](https://movingai.com/benchmarks/grids.html)
and its [map and scenario format](https://movingai.com/benchmarks/formats.html).
It records the source files and map hashes in the manifest. Dataset files and
campaign output stay under the ignored `benchmark-results/` directory.

The parser accepts scenario headers `version 1` and `version 1.0`, keeps
`x + width*y` node IDs, and rejects scaled scenarios. Movement uses cardinal
cost 1, diagonal cost sqrt(2), and requires both side cells to be passable for
a diagonal. The parser rejects symbols outside the profile instead of
silently reclassifying them.

## Build and run

Download the map and scenario archives linked by the official benchmark page,
extract each family into its own directory, then run:

```text
python3.12 setup.py build_ext --inplace \
  --build-temp /tmp/pathplanning-native-build/temp \
  --build-lib /tmp/pathplanning-native-build/lib
python3.12 scripts/benchmark_shortest_path.py prepare \
  --dataset-root benchmark-results/datasets/movingai-v2 \
  --profile pilot \
  --manifest benchmark-results/pilot_manifest.json
python3.12 scripts/benchmark_shortest_path.py validate \
  --manifest benchmark-results/pilot_manifest.json
python3.12 scripts/benchmark_shortest_path.py run \
  --manifest benchmark-results/pilot_manifest.json \
  --campaign benchmark-results/pilot \
  --pass work
python3.12 scripts/benchmark_shortest_path.py run \
  --manifest benchmark-results/pilot_manifest.json \
  --campaign benchmark-results/pilot \
  --pass latency --scope public_api --graph-state reused_graph
python3.12 scripts/benchmark_shortest_path.py run \
  --manifest benchmark-results/pilot_manifest.json \
  --campaign benchmark-results/pilot \
  --pass memory
python3.12 scripts/benchmark_shortest_path.py analyze \
  --campaign benchmark-results/pilot
```

`prepare` freezes the selected map/query identities, source hashes, variant
definitions, and work/latency/memory cohorts. The pilot chooses up to three
maps in each available family and at most 20 work queries per map and cost
quantile; the latency and memory cohorts sample at most four of those queries
per quantile. `validate` runs the independent Python Dijkstra oracle before
candidate execution and fails on disagreement with the scenario reference.
Oracle work runs in deterministic batches of 16, with at most eight worker
processes. Each completed batch is appended to `<manifest-stem>.oracle.jsonl`,
so an interrupted validation can resume from cached workload IDs without
repeating completed queries. Candidate measurement passes remain sequential.
The selected scenario files serialize diagonal movement with `1.414213562`;
validation accounts for that archive precision separately from the exact
`sqrt(2)` used to check candidate paths.

The runner retains warmups, errors, and timeouts in JSONL. Re-running the same
command resumes only missing observation keys; changing the manifest, binary,
protocol, schedule seed, timeout, or source identity is rejected for an
existing pass configuration. `runs.jsonl`, saved schedules, pass configs,
`manifest.json`, summary, and report make up the campaign record.

## Measurement boundaries

Latency uses the production `_search_engine` extension with `PP_ENABLE_METRICS`
unset. The metrics extension is built from a separate object directory and is
called only in the `work` pass. That pass clones the production graph before
measurement, then compares success, stop reason, exact path IDs, cost, iters,
and nodes against the release call. Clone time and instrumented call time are
reported separately. The `memory` pass uses a fresh worker per query so RSS is
a process lifetime high-water; it is not subtracted from another high-water.

Use `prepared_kernel` to time only a prepared native call, or `public_api` to
include adapter, heuristic preparation, result conversion, and freeing.
`fresh_graph` includes graph creation per invocation; `reused_graph` prepares
one graph per variant and reuses it. Reverse CSR setup is measured separately
and is retained only on the bidirectional graph that prepared it.

The release and metrics libraries share the same algorithm sources and search
ordering. Work and allocation hooks compile out of production, and state and
frontier types resolve to their normal `std::vector`/`std::deque` types in the
release build. Native allocation reports cover search state, frontier,
reconstruction, and result path buffers; allocator metadata and retained graph
storage are reported separately. RSS is process-wide and includes Python,
NumPy, map loading, and native graph creation.

The Markdown report separates the three measurement boundaries. Latency reports
per-query median/P95 and paired speedups from release repeats. Work reports
median/P95 search counters and the separately instrumented query workspace,
state, frontier, path, and result buffers. The fresh-worker memory pass reports
process RSS alongside input occupancy and retained CSR storage. RSS includes
Python, NumPy, map loading, and graph creation; it is not query-only memory.
`summary.json` retains every available metric distribution (count, median, P95,
range, and IQR), paired ratios, and bootstrap intervals. Each report states
executed observation counts and missing/error outcomes. Profile JSON files
specify the pilot and scaling configurations; a profile definition alone is
not a measured scaling campaign.
