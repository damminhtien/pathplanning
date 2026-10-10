# Reproducing the single-agent shortest-path benchmark

This runner implements the MovingAI `land_octile_v1` profile from the
[Moving AI 2D benchmark collection](https://movingai.com/benchmarks/grids.html)
and its [map and scenario format](https://movingai.com/benchmarks/formats.html).
It records the source files and map hashes in the manifest. Dataset files and
campaign output stay under the ignored `benchmark-results/` directory. See
[`docs/benchmarks/dataset_installation.md`](benchmarks/dataset_installation.md)
for the complete DIMACS, BARN, MovingAI, Monash, and OMPL dataset inventory.

The parser accepts scenario headers `version 1` and `version 1.0`, keeps
`x + width*y` node IDs, and rejects scaled scenarios. Movement uses cardinal
cost 1, diagonal cost sqrt(2), and requires both side cells to be passable for
a diagonal. The parser rejects symbols outside the profile instead of
silently reclassifying them.

![Examples of installed benchmark inputs across MovingAI, DIMACS, Monash, BARN, and OMPL; separate panels show distinct semantics and scales](benchmarks/images/benchmark_dataset_examples.png)

## Latest completed campaign snapshot

The 2026-10-10 refresh contains 216,535 measurement entries in separate
MovingAI land, scaling, unreachable, grid-specialist, DIMACS, voxel, and BARN
cohorts. It covers all 789 MovingAI maps; the format runner keeps road-cost,
voxel-movement, and BARN point-robot semantics separate. OMPL.app resources
and weighted terrain are installed but have no compatible planner cohort yet.
The task-model distinctions and source exclusions are in the
[dataset characterization](benchmarks/dataset_characterization.md).

| Campaign | Executed workloads | Outcome boundary |
|---|---:|---|
| MovingAI land | 3,363 work queries × 12 variants | All 40,356 observations returned valid paths; latency uses 788 map/query identities × 12 × 7 scheduled runs; memory uses 30 × 12. |
| MovingAI unreachable | 2,160 queries across 180 maps × 12 variants | All 25,920 observations correctly proved the queries unreachable; separate from reachable-scenario coverage. |
| Scaling | 1,600 work queries × 17 variants | 1,538 reachable and 62 unreachable; latency uses 400 × 17 × 7 scheduled runs; memory uses 80 × 17. |
| MovingAI grid specialists | 788 map/query pairs × 4 planners | 3,114 valid results; D* Lite was resource-limited on 38 pairs. |
| DIMACS / voxels | 13 road graphs and 2 distinct voxel maps × 13 variants | Six distance, six travel-time and one source-weight graph; 20 voxel paths optimal and 6 suboptimal. |
| BARN | 300 worlds × 14 compatible planners | 2,673 valid paths and 1,527 no-solution outcomes; no invalid paths or planner errors. |

These counts are execution coverage, not a cross-family leaderboard. The README
figures are regenerated from the separate full-refresh campaigns; single-call
dataset observations stay separate from repeated latency cohorts.
Repeated timing runs used an Apple M2 Max with macOS 27.0.1 and Python 3.14.8;
interpret these timings within that host and protocol.

## Build and run

Install the surveyed collection once. The installer places MovingAI land maps
and their scenarios beside one another in the family directories expected by
the existing parser; terrain and voxel inputs remain in separate directories.
Then build and prepare the pilot:

```text
python3.12 scripts/install_benchmark_datasets.py install \
  --catalog benchmark-results/dataset-characterization/catalog.json
python3.12 setup.py build_ext --inplace \
  --build-temp /tmp/pathplanning-native-build/temp \
  --build-lib /tmp/pathplanning-native-build/lib
python3.12 scripts/benchmark_shortest_path.py prepare \
  --dataset-root benchmark-results/datasets/movingai-v2 \
  --profile pilot \
  --manifest benchmark-results/pilot_manifest.json
python3.12 scripts/benchmark_shortest_path.py validate \
  --manifest benchmark-results/pilot_manifest.json
python3.12 scripts/benchmark_shortest_path.py augment-unreachable \
  --manifest benchmark-results/pilot_manifest.json \
  --output benchmark-results/pilot_with_unreachable_manifest.json \
  --per-map 10 --seed 7
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
python3.12 scripts/benchmark_shortest_path.py run \
  --manifest benchmark-results/pilot_with_unreachable_manifest.json \
  --campaign benchmark-results/pilot_unreachable \
  --pass work --cohort unreachable
```

## Regenerate the README scenario gallery

After the work pass has produced `runs.jsonl`, install the optional rendering
dependencies and replay the selected real map/scenario pairs:

```text
python3.12 -m pip install -e ".[viz]"
python3.12 scripts/generate_movingai_scenario_assets.py \
  --dataset-root benchmark-results/datasets/movingai-v2 \
  --campaign benchmark-results/pilot/runs.jsonl \
  --output-dir assets/images
```

The generator selects the median path-length valid A* scenario from each of
five MovingAI map families, checks the recorded map hash and scenario
endpoints, and runs the displayed planner with a bounded trace for the image.
These offline trace runs do not enter the latency pass. The optional `viz`
dependencies include Matplotlib and SciencePlots; the figures use SciencePlots'
`science` style with LaTeX rendering disabled. The raw dataset and campaign
remain under the ignored `benchmark-results/` directory.

`prepare` freezes the selected map/query identities, source hashes, variant
definitions, and work/latency/memory cohorts. The pilot chooses up to three
maps in each available family and at most 20 work queries per map and cost
quantile; latency samples up to four of those queries per quantile, while memory
uses one query on up to three map-size quantiles per family. The `coverage`
profile instead selects a query from each available displacement bin on every
map with source scenarios. `validate` runs the independent Python Dijkstra
oracle before candidate execution and fails on disagreement with the scenario
reference. Synthetic scaling workloads are reference-checked against Dijkstra
without a source-scenario optimum.
Oracle work runs in deterministic batches of 16, with at most eight worker
processes. Each completed batch is appended to `<manifest-stem>.oracle.jsonl`,
so an interrupted validation can resume from cached workload IDs without
repeating completed queries. Candidate measurement passes remain sequential.
The selected scenario files serialize diagonal movement with `1.414213562`;
validation accounts for that archive precision separately from the exact
`sqrt(2)` used to check candidate paths.

`augment-unreachable` copies a successfully validated manifest and adds up to
ten seeded negative queries per selected map with multiple free-space
components. The component test uses cardinal connectivity, which matches the
no-corner-cutting octile movement model. These rows are oracle-checked and run
only when `--cohort unreachable` is requested; the source-scenario workload
cohort and its work, latency, and memory denominators stay unchanged. Maps with
one component are recorded as having no negative query available.

To cover every MovingAI land map that has a valid source scenario, prepare the
`coverage` profile. It scans all scenario rows and keeps one hash-selected query
per map and normalized-displacement bin. The work pass runs those queries; the
latency pass keeps one query per map; the fresh-process memory pass samples up to
three map-size quantiles per family. This provides full map coverage without
running all 1.7 million source scenario rows as if they were independent maps:

```text
python3.12 scripts/benchmark_shortest_path.py prepare \
  --dataset-root benchmark-results/datasets/movingai-v2 \
  --profile coverage \
  --manifest benchmark-results/coverage_manifest.json
python3.12 scripts/benchmark_shortest_path.py validate \
  --manifest benchmark-results/coverage_manifest.json
python3.12 scripts/benchmark_shortest_path.py run \
  --manifest benchmark-results/coverage_manifest.json \
  --campaign benchmark-results/coverage --pass work
```

Scaling profiles are generated, hash-pinned, and validated separately. The
default profile has size, obstacle-density, and room/serpentine corridor-width
sweeps, plus five consistent heuristic scales. The common 512-by-512, 20%
random map/query set is shared across the size and density cohorts and is marked
with both memberships rather than counted as independent data:

```text
python3.12 scripts/benchmark_scaling.py prepare \
  --manifest benchmark-results/final/scaling/manifest.json
python3.12 scripts/benchmark_scaling.py validate \
  --manifest benchmark-results/final/scaling/manifest.json
python3.12 scripts/benchmark_scaling.py run \
  --manifest benchmark-results/final/scaling/manifest.json \
  --campaign benchmark-results/final/scaling --pass work
python3.12 scripts/benchmark_scaling.py run \
  --manifest benchmark-results/final/scaling/manifest.json \
  --campaign benchmark-results/final/scaling --pass latency
python3.12 scripts/benchmark_scaling.py run \
  --manifest benchmark-results/final/scaling/manifest.json \
  --campaign benchmark-results/final/scaling --pass memory
python3.12 scripts/benchmark_scaling.py analyze \
  --campaign benchmark-results/final/scaling
```

The runner retains warmups, errors, and timeouts in JSONL. Re-running the same
command resumes only missing observation keys; changing the manifest, binary,
protocol, schedule seed, timeout, or source identity is rejected for an
existing pass configuration. `runs.jsonl`, saved schedules, pass configs,
`manifest.json`, summary, and report make up the campaign record.

## Run the other installed dataset formats

Use the source-aware runner to exercise compatible graph, voxel, and geometric
algorithms while retaining each format's movement, cost, collision, and oracle
semantics. It resumes missing dataset/algorithm observations and writes
`runs.jsonl`, `inventory.json`, and `summary.json` beneath the campaign. The
independent graph oracle uses the optional SciPy extra, installable with
`pip install -e ".[scipy]"`.

```text
python3.12 scripts/benchmark_format_cohorts.py run \
  --dataset-root benchmark-results/datasets \
  --dataset-index benchmark-results/datasets/index.json \
  --output benchmark-results/final/format_cohorts \
  --sources dimacs voxels barn
```

The DIMACS cohort uses one seeded reachable query per graph within configured
size limits. Voxel cohorts use one source-scenario query per eligible map and
keep MovingAI/Monash byte-identical mirrors separate in the inventory without
double-counting them. BARN is a derived point-robot XY cohort with one
collision-checked query per world, exact point goals, and exact segment-circle
collision validation. The native planner samples at `collision_step=0.01` m
with each circle inflated by half a sample step and a 1 nm numerical guard,
which prevents the sampling check from accepting a path through a source
obstacle. Its supplied path arrays are not interpreted as world coordinates.
JIT* is excluded because this point-robot model has no kinematic Jacobian for
manipulability scoring. The BARN campaign is a single-run path-validity and
work-coverage pass, not a latency comparison. OMPL.app resources need the
matching mesh collision-checking runtime, and MovingAI terrain needs a terrain
cost table before it can be benchmarked as weighted terrain.
These exclusions are recorded in the summary rather than pooled into another
cohort.

The coverage manifest's one-query-per-map cohort also supports grid-specialist
algorithms that require the public 2D grid API:

```text
python3.12 scripts/benchmark_movingai_grid_specialists.py run \
  --dataset-root benchmark-results/datasets/movingai-v2 \
  --manifest benchmark-results/final/movingai_coverage_manifest.json \
  --output benchmark-results/final/movingai_grid_specialists
```

JPS and D* Lite are checked against the independent no-corner-cutting octile
oracle. Theta* and Lazy Theta* are collision-checked in a separate any-angle
cohort and are not ranked against the octile optimum. JPSW requires the missing
terrain cost table, which the runner records as an exclusion.

## Regenerate the dataset illustration

The multi-format figure reads the installed source assets and uses only the
Python standard library. To regenerate the SVG and a PNG for Markdown previews,
install `rsvg-convert` and run:

```text
python3.12 scripts/render_benchmark_dataset_examples.py \
  --dataset-root benchmark-results/datasets \
  --output docs/benchmarks/images/benchmark_dataset_examples.svg \
  --png-output docs/benchmarks/images/benchmark_dataset_examples.png
```

It shows a named MovingAI scenario rendering, a small induced DIMACS road
subgraph, one slice from each of two 3D maps, BARN's source cylinders, and an
OMPL image resource. It does not create planner observations or infer graph
difficulty from visual density.

## Refresh and regenerate the README benchmark figures

The README figure set is built from separate full-refresh campaigns. The
following sequence validates the frozen MovingAI coverage manifest, measures
the 2D land work/latency/memory passes, and analyzes them. The scaling, format,
grid-specialist, and unreachable-query commands use the same refresh root so
the figure generator reads one consistent result set. Existing completed
manifests can be reused because their dataset and workload hashes are pinned.

```text
python scripts/benchmark_shortest_path.py validate \
  --manifest benchmark-results/final/movingai_coverage_manifest.json
python scripts/benchmark_shortest_path.py run \
  --manifest benchmark-results/final/movingai_coverage_manifest.json \
  --campaign benchmark-results/refresh_20261010/movingai_land --pass work
python scripts/benchmark_shortest_path.py run \
  --manifest benchmark-results/final/movingai_coverage_manifest.json \
  --campaign benchmark-results/refresh_20261010/movingai_land \
  --pass latency --repeats 5 --warmups 2
python scripts/benchmark_shortest_path.py run \
  --manifest benchmark-results/final/movingai_coverage_manifest.json \
  --campaign benchmark-results/refresh_20261010/movingai_land --pass memory
python scripts/benchmark_shortest_path.py analyze \
  --campaign benchmark-results/refresh_20261010/movingai_land
python scripts/benchmark_scaling.py run \
  --manifest benchmark-results/final/scaling/manifest.json \
  --campaign benchmark-results/refresh_20261010/scaling --pass work
python scripts/benchmark_scaling.py run \
  --manifest benchmark-results/final/scaling/manifest.json \
  --campaign benchmark-results/refresh_20261010/scaling \
  --pass latency --repeats 5 --warmups 2
python scripts/benchmark_scaling.py run \
  --manifest benchmark-results/final/scaling/manifest.json \
  --campaign benchmark-results/refresh_20261010/scaling --pass memory
python scripts/benchmark_scaling.py analyze \
  --campaign benchmark-results/refresh_20261010/scaling
python scripts/benchmark_format_cohorts.py run \
  --dataset-root benchmark-results/datasets \
  --dataset-index benchmark-results/datasets/index.json \
  --output benchmark-results/refresh_20261010/format_cohorts \
  --sources dimacs voxels barn
python scripts/benchmark_movingai_grid_specialists.py run \
  --dataset-root benchmark-results/datasets/movingai-v2 \
  --manifest benchmark-results/final/movingai_coverage_manifest.json \
  --output benchmark-results/refresh_20261010/grid_specialists
python scripts/benchmark_shortest_path.py augment-unreachable \
  --manifest benchmark-results/final/movingai_coverage_manifest.json \
  --output benchmark-results/refresh_20261010/movingai_all_algorithms_with_unreachable_manifest.json \
  --per-map 10 --seed 7
python scripts/benchmark_shortest_path.py run \
  --manifest benchmark-results/refresh_20261010/movingai_all_algorithms_with_unreachable_manifest.json \
  --campaign benchmark-results/refresh_20261010/movingai_unreachable_full \
  --pass work --cohort unreachable
```

After all campaigns finish, install the optional plot dependencies with
`python -m pip install -e ".[viz]"` and render all four cohort figures:

```text
python scripts/generate_full_benchmark_assets.py \
  --campaign-root benchmark-results/refresh_20261010 \
  --output-dir assets/images
```

The generator reads saved summaries and JSONL observations only; it does not
run planners. Repeated latency panels use the median of five measured calls per
query after two warm-ups, followed by the median and P95 across queries. DIMACS,
voxel, BARN, and grid-specialist panels use one call per independent instance;
their P95 describes workload spread and is kept separate from repeat-based
latency. The work/quality figure uses the 2D octile oracle, while the outcome
figure preserves each format's own success/status categories. The memory figure
shows separately measured query workspace and fresh-worker RSS, which includes
the interpreter, input loading, and graph setup.

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
