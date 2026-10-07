# Single-Agent Shortest-Path Evaluation Plan

Review date: 2026-10-04. Plan version: 2.

Status: proposed for implementation. This document supersedes the plan from the discussion and fixes measurement definitions, work items, and acceptance criteria. Files and commands marked “planned” have not been implemented. This document contains no new benchmark results.

## 1. Review findings and decisions

| Previous plan item | Decision after review |
| --- | --- |
| Use the existing grid for MovingAI | Build benchmark CSR from occupancy using the no-corner-cutting rule. The existing grid checks only source and destination cells and may produce an optimum that differs from the scenario. |
| Use native Dijkstra as the oracle | Use an independent reference Dijkstra over the original occupancy grid. Native Dijkstra is one of the evaluated algorithms. |
| Treat graph_init_s as graph creation time | The current field includes the adapter, query mapping, and heuristic preparation; the graph may be reused. Measure these boundaries separately. |
| Infer memory from expanded | SearchStates allocates arrays for all node slots. Record slots, capacity, and live allocations. |
| Apply reopen/re-expansion counters to every kernel | The current kernel skips CLOSED nodes and does not support reopening. Record capability and null for inapplicable metrics. Separate repeated expansions across anytime passes. |
| Include reverse-edge caching in query time | Reverse CSR is created lazily and then retained by the graph. Measure preparation, first query, reuse, and ownership for each variant. |
| Seed + repeats are sufficient for fairness | Specify timer scope, graph state, heuristic mode, paired schedule, statistical unit, and timeout handling. |
| Subtract two ru_maxrss values to estimate query peak | ru_maxrss is a lifetime high-water mark. Report process peak; measure native memory and sampled RSS windows separately. |
| max_expansions is the total anytime budget | The current limit applies to each pass. Add a separate total-budget contract before using quality-versus-budget analysis. |
| Change global SCHEMA_VERSION to v2 | Add a separate v2 builder; keep the v1 runner and historical reports on their existing structure. |

The source was inspected in the worktree on the review date; the observed HEAD was c07cef0cb53ad9bbe0ad8cc13672c3fb1869a8e9. Before implementation, recheck HEAD, the diff, and the ABI because other work may be in progress. These are source-based observations and have not been verified by running the binary.

The integration points reviewed:

- [Grid2DSearchSpace](../pathplanning/spaces/grid2d.py): movement, heuristic, and to_native_graph.
- [NativeGraph](../pathplanning/native/graph.py): CSR reuse, labels, and ownership.
- [Search adapter](../pathplanning/planners/search/_internal/native.py): query preparation, timers, and conversion back to Python.
- [Search kernel](../pathplanning/native/search_engine.cpp): states, heap/deque, reverse CSR, consistency validation, and anytime passes.
- [Search ABI](../pathplanning/native/search_engine.h), [ABI version](../pathplanning/native/abi_version.h), [FFI loader](../pathplanning/native/_ffi.py).
- [Benchmark contract v1](benchmark_contract.md), [example runner](../scripts/benchmark_planners.py), [trace benchmark](../scripts/benchmark_trace_overhead.py).

## 2. Scope and algorithm configurations

MVP: static 2D grids, exact start/goal, and positive costs. The core set is Dijkstra, A*, bidirectional Dijkstra, and bidirectional A*. The quality trade-off group adds weighted A* with weights 1.25, 1.5, and 2.0, plus greedy best-first on the same inputs. Weight 1.0 must match A*.

BFS has a separate 4-connected, unit-cost suite with its own oracle; octile optimal cost does not apply to that suite. DFS is only a baseline for finding a path. Anytime belongs to a later milestone, after pass observations and a meaningful total budget exist. Sampling, 3D, dynamic, JPS, and any-angle tracks remain in section 12.

Each observation records a measurement scope:

- **prepared_kernel**: graph and query arrays are prepared; the timer brackets the native call itself.
- **public_api**: the timer brackets plan_discrete, including the adapter, heuristic preparation, native call, and result conversion/free.

Each scope supports **fresh_graph** or **reused_graph**. These names describe graph ownership/cache state; they do not guarantee cold or warm CPU/file caches. The default headline is public_api + reused_graph; prepared_kernel explains the underlying mechanism.

## 3. Dataset, representation, and manifest

### 3.1. MovingAI profile

Use the 2D version 2 collection. The initial scenario parser supports version 1 or 1.0 headers; collection version and scenario-format version are separate concepts. According to the [official format](https://movingai.com/benchmarks/formats.html), a scenario has nine fields, coordinates start at the top-left, and the optimum uses diagonal cost sqrt(2) with corner cutting prohibited.

Profile **land_octile_v1**:

- Cardinal cost is 1; diagonal cost is sqrt(2), and a diagonal move is valid only when both adjacent cardinal cells are traversable.
- Preserve the original ASCII symbols. Land agents can traverse `.`, `G`, and `S`; `@`, `O`, `T`, and `W` are blocked. `W` has separate water-agent semantics in the format and is outside this profile.
- An unknown symbol produces dataset_error; do not silently convert it to terrain or an obstacle.
- Validate the header, row and column counts, map path, bounds, endpoint passability, and that the optimum is finite and nonnegative.
- The MVP rejects scenarios whose dimensions differ from the map with unsupported_scaled_scenario. The original format allows scaling, so this limitation must be documented.
- Resolve the map path for each scenario row; do not assume that a scenario file refers to only one map.

CSR uses stable IDs **id=y*width+x** and **N=width*height** slots, including empty rows for blocked cells; **V_free** is the number of traversable cells; **E** is the number of valid directed edges. Report both N and V_free. Build CSR according to the profile; do not use the current grid factory to infer MovingAI optima.

Keep the current runner’s order of the eight motion directions fixed. Record each variant’s tie-breaking policy: best-first currently orders by increasing f, then increasing h, then increasing insertion order. Do not require different algorithms to return the same path when multiple optimal paths exist.

The main heuristic cohort uses octile distance, stored as a vectorized float64 array for all N slots for each goal; Dijkstra uses h=0. Keep **precomputed_array** at every size. The grid adapter currently changes heuristic preparation around 65,536 slots, so changing the heuristic mode must be a separate ablation. For prepared_kernel, heuristic preparation is outside the call timer but its time and bytes are still reported; public_api includes it in query cost. The initial cohort does not cache h-arrays across queries; any additional caching is a separate variant.

The planned workloads.py contains MovingAIGrid, whose to_native_graph returns the exact CSR already built and whose native_heuristic_values returns the vectorized octile array. Each variant has its own instance/handle. The public API uses this adapter; prepared_kernel uses the same CSR/start/goal/h-arrays through a prepared FFI call. Set the node materialization limit explicitly to N in the problem parameters when using the factory adapter, instead of accidentally rejecting a 1024x1024 map because of the default limit of 1,000,000.

### 3.2. Planned data profiles

| Profile | Specific selection |
| --- | --- |
| fixtures | Maps from 2x2 to 16x16: one- and two-sided corner blocking, corridors, disconnected regions, start=goal, and tied paths; also directed graphs and stale heap entries. Include hand-counted costs/counters. |
| pilot | Six families: DAO, Starcraft, room, maze, random, and street. Select three maps per family by low/median/high V_free, breaking ties by name; at most 100 scenarios per map: five C* quantiles × up to 20 rows, selected by hash with workload_seed=7. At most 1,800 queries. |
| full | All maps/scenarios from the six chosen families, with the file list and hashes frozen. Record parser exclusions with reasons and counts. |
| scaling | Generators and sweeps from section 9, with fixed seeds and versions. |

These families are in the [MovingAI 2D catalog](https://movingai.com/benchmarks/grids.html). Record missing maps/bins in the manifest; do not replace them based on candidate success. If the three map positions resolve to the same map, select the nearest distinct map using the fixed ordering. Attach baseline-expanded difficulty only after query selection, outside the measurement window.

Pilot correctness/work uses all selected queries, up to 1,800. Pilot latency/RSS uses a fixed subset of at most 20 queries per map: select four from each chosen bin with the same hash rule, up to 360 queries. The initial primary latency campaign runs only public_api + reused_graph; other scopes/graph states use a separate ablation campaign on this subset. For eight MVP variants, one latency campaign has at most 20,160 measured observations + 5,760 warmups; work has at most 14,400 observations; RSS has at most 2,880 workers. These are protocol estimates, not completed experiments. Freeze and record the full latency/memory cohort in a manifest before running, based on pilot cost; do not generate the full Cartesian product of scopes, variants, and modes automatically.

Manifest/query fields: family, map/scenario URL and SHA-256, scenario row, movement profile, dimensions, N/V_free/E, density, start/goal, original optimum string, oracle C*, h(start), solution depth, and baseline A* expansions. Record generator version, bin boundaries, and seeds.

**workload_id** depends on data and problem semantics; **variant_id** depends on algorithm/parameters/heuristic mode/tie policy; **experiment_id** adds source/binary/host/protocol. Pair by workload_id and protocol; experiment_id from a different build is not a pairing key.

Keep dataset payloads in the local cache; version metadata, attribution, and profiles. Results go in benchmark-results/, which the current contract excludes from the source fingerprint.

## 4. Oracle and correctness

Reference Dijkstra uses Python heapq over the original occupancy grid and enumerates moves independently; it does not call NativeGraph, grid.neighbors, or the search kernel. Cache by map hash + movement profile + start/goal + reference version. Native Dijkstra is a candidate/baseline; run the reference outside the timer/RSS window.

The validator checks endpoints, adjacency, obstacles, and the corner rule, then calculates cost independently from the cardinal/diagonal move counts. Compare declared_cost with the independently calculated cost, then compare it with the oracle. Candidate-vs-oracle tolerance: atol=1e-8, rtol=1e-10. For scenario-vs-oracle, first allow floating-point error and one unit in the last place. MovingAI scenario optima in the dataset accumulate diagonal costs using `1.414213562`; if that representation differs from `sqrt(2)`, the validator derives the unique cardinal/diagonal step counts from the oracle C*, then compares the scenario optimum string using the same constant and half a unit in the last place. This representation tolerance does not apply to candidate-vs-oracle comparisons.

A scenario mismatch outside tolerance is a dataset/reference discrepancy: investigate it before benchmarking and do not automatically attribute it to the candidate. Compare different algorithms by path validity/cost. For the same algorithm/input, release-vs-metrics must match on stop reason, path hash, cost, iters, and nodes.

Store execution_status, planner_stop_reason, path_present, path_valid, declared_cost_matches, and optimal_cost_matches separately. Timeout/crash/invalid_path/valid_suboptimal/proved_unreachable are distinct outcomes. For start=goal, C*=0: use an absolute check; ratio/gap is null. An unreachable query confirmed by the oracle is not a failure when the candidate correctly returns “no path.”

## 5. Work metrics and measurement points

Counters are uint64 in C/C++ and int in JSON. Do not route them through PlanResult.stats, which is currently Mapping[str,float]. An observation contains input/outcome/work/memory/timing/capabilities/provenance. **0** means measured and did not occur; **null** means not applicable/not measured, with a reason.

| Planned field | Definition and increment point |
| --- | --- |
| expanded | A node processed after discarding CLOSED entries, including the goal when the kernel actually processes it; preserves the current meaning of iters. |
| discovered_first | g changes from infinity to finite, including the start. Bidirectional search records each side; summed side counts are not the node union. |
| edges_examined | Edges read in the search adjacency loop, before checking/skipping them. |
| relaxation_attempts | Tentative-cost calculations on valid edges. |
| relaxation_successes | Updates to g/parent, separating first discoveries from improvements to known labels. |
| closed_neighbor_skips | An edge to a CLOSED node is skipped; do not call this dominance pruning. |
| nonimproving_skips | A candidate does not improve the current label; greedy has its own skip reason based on its criterion. |
| frontier_pushes / frontier_pops | Actual heap/queue/deque operations, including initial pushes and stale pops. |
| stale_pops | An entry is skipped because its node is CLOSED; denominator is frontier_pops. |
| frontier_peak_entries | Maximum live entries, distinct from unique OPEN nodes; record side peaks and the simultaneous total peak. |
| heuristic_array_values_prepared | Values prepared before search, as part of query preparation. |
| heuristic_lookups / computations | Separate reads from the h-array and actual h computations; validation has its own phase. |
| validation_edge_checks | Full CSR consistency scan in bidirectional A*; do not mix with search-loop edges_examined. |
| goal_tests | Goal checks in the kernel; goal-mask preparation in Python is a separate field. |
| reopen_count / reexpanded_same_pass | null when supports_reopen=false in the current implementation. |
| anytime pass fields | Counters per pass, total work across passes, and maximum live memory; current anytime nodes are the sum of discoveries across passes. |

Derived ratios: expanded/V_free, edges_examined/E, successful/attempted relaxations, stale_pops/frontier_pops, pushes/expanded. A zero denominator yields null. Do not add these categories into a single “total operations” count because edge checks, heap pushes, and h-computations have different costs. heap_comparisons is an extension after the MVP.

## 6. Timing and memory boundaries

### 6.1. Timing

| Field | Boundary |
| --- | --- |
| input_load_s | Read/parse input, at dataset/map level. |
| graph_build_s | Create CSR and copy/adopt the native handle. |
| algorithm_prepare_s | Prepare reverse CSR or other variant-retained data. |
| query_prepare_s | Map the query and prepare the h-array, goal flags, and options. |
| native_call_s | C/C++ call on prepared inputs; includes required validation, state initialization, search, and native path construction. |
| result_decode_free_s | Convert/copy the path to Python and free native output. |
| api_total_s | Bracket public plan_discrete on the declared graph state. |

Adjacent scopes use a shared timestamp to check totals and residuals; nested scopes must not be double-counted. Cloning CSR between libraries is setup before the prepared_kernel timer and has its own cost. Process startup/harness overhead is recorded outside algorithm timers.

If state_init_s/search_loop_s/path_reconstruct_s are needed, use native profiling and label it diagnostic_profile. These phase timings explain the mechanism; the latency headline comes from release builds with timer overhead declared.

### 6.2. Memory

| Field | Measurement |
| --- | --- |
| input_occupancy_bytes | ndarray.nbytes; keep labels/maps/parse buffers alive in separate fields. |
| base_csr_capacity_bytes | Capacity of offsets/indices/costs multiplied by sizeof; includes blocked slots. |
| prepared_retained_bytes | Reverse CSR and precomputed tables retained after preparation. |
| query_input_bytes | h-array, goal flags, and options. |
| state_slots_allocated / parent_id_bytes | Actual slots in each SearchStates and parent width (32/64-bit). |
| state_capacity_bytes | Capacities of g/parent/flags multiplied by sizeof. |
| frontier_capacity_bytes_peak | Heap vector capacity; deque requires allocator tracking. |
| native_requested_bytes_peak | Counting allocator for declared structures/buffers; includes old/new overlap during reallocation; record allocation coverage. |
| query_workspace_peak_bytes | Maximum sum of simultaneously live workspace bytes, separate from retained/input/output. |
| result_path_bytes | Native path and Python copy, with ownership/lifetime recorded separately. |
| process_peak_rss_bytes | OS/unit-normalized ru_maxrss in a fresh worker; includes import/load/build/query. |
| query_peak_rss_sampled_bytes | Optional external sampler in the READY→DONE window; record interval/resolution. Short queries may miss spikes. |

Total peak is **max_t(sum live component bytes at t)**, not the sum of component peaks. Capacity/requested bytes differ from resident RSS and do not include all allocator overhead. Label incomplete coverage.

The RSS campaign uses a fresh release worker for each map+variant+query and retains one candidate graph. Report lifetime peak; record current RSS before the query if the backend supports it. Do not subtract two high-water marks to infer query peak. Keep RSS sampling separate from the latency campaign.

Report absolute bytes, bytes/V_free, and bytes/N. One side’s state payload with 32-bit parents is approximately (8+4+1)*N before headers/capacity; bidirectional search has two sets. Native tracking must verify lifetimes/allocations; do not infer memory from expanded.

## 7. Run protocol and statistics

Default pilot: 2 warmups/query/variant, 7 measured repeats, schedule_seed=7. Default full campaign: 15 measured repeats; if the pilot shows unstable timing, freeze a new repeat count before the full run. These are initial settings and do not guarantee a particular statistical precision.

Latency workers are grouped by map, with a separate graph handle for each variant; only one query computes at a time. A block consists of query and repeat; shuffle variant order using schedule_seed and save the schedule for replay. When multiple graphs are resident, record the latency worker’s total retained footprint; take headline memory measurements from a separate worker.

fresh_graph creates a new handle for every invocation and reports build/first-query costs. reused_graph prepares each variant’s graph and reverse CSR separately before warmup, then retains them across queries; current search states are still recreated on each call. Do not share one variant’s cache with another.

The controller starts the query watchdog after READY, once setup is complete: default pilot limits are 5 s/query and 60 s/setup, with separate timeout stages. Record elapsed time as a lower bound; treat timeout as censored when calculating speedup. Leave max_expansions unset for the MVP headline; resource-budget sweeps use a separate protocol. Deterministic search uses the same input/parameters for each repeat; schedule_seed differs from workload_seed.

Collect work metrics once per query/variant; repeat checks on fixtures and a pilot sample. Compare release iters/nodes with the metrics build. Retain all warmups but exclude them from summaries.

For query i, t_i is the median across measured release repeats. Report the median/p95 of t_i across queries and the IQR/dispersion across repeats. The p95 across queries is not the noise p95 from seven repeats of one query.

Speedup s_i=t_baseline_i/t_candidate_i on common-valid-solved queries under the same protocol; report median, geometric mean, and CI. Bootstrap by map cluster then query within each map, preserving pairs: default 2,000 draws, bootstrap_seed=7. With few maps, state CI limitations and show per-map results. Repeats/query within the same map are not fully independent samples.

Report micro results by query and macro results by map/family with explicit weights. Coverage = valid solved unique queries / oracle-solvable queries in the frozen cohort, separate from execution-repeat counts. start=goal belongs to the solvable cohort; reference-unreachable cases have a separate decision-correctness denominator over all valid inputs. Determine eligibility from the input/oracle before candidate runs. Report optimal/invalid/crash/timeout counts, mean quality ratio, and ratio of sums with denominators. Show common-solved quality/speedup next to coverage over the full manifest. Flag and investigate repeats with inconsistent outcomes before publication.

Provenance includes source/binary/input hashes, OS/CPU/RAM, actual compiler/flags, timer/resolution, h-mode, graph state, seeds, and actual order. Build logs confirm flags; compiler configuration metadata alone is insufficient. Run sequentially, and record whether affinity/frequency control was available and observed system load.

## 8. Native instrumentation and anytime

### 8.1. Native metrics

Add a planned **_search_metrics_engine** extension from the same search_engine.cpp, using C++17/-O3, PP_ENABLE_METRICS=1, and a separate object directory as configured in [setup.py](../setup.py). Compile metric hooks out of release builds; do not infer counters from traces.

Planned search_metrics.h: uint64 counters, memory/capabilities, struct_size, and a separate metrics ABI. Entry point pp_native_search_plan_measured returns SearchResult and an independent metrics output. Context lifetime is per call/pass; the heap adapter preserves comparator and insertion order. Attach the counting allocator only to the metrics build. Changing heap/reserve policy is a separate optimization variant and requires an ablation.

Add load_search_metrics_library to _ffi.py with ABI/export/layout checks. Keep production SearchResult/PlanResult layouts unchanged by using a separate measurement API. Increment the search ABI version only when exported declarations actually change; do not hardcode the next ABI version from this review.

The metrics library creates/frees its own graph handles and results. Export a CSR view and copy it into the library before the timer; keep the owner alive during the copy. Do not pass opaque handles or free buffers across libraries.

Add planned APIs **pp_graph_prepare_reverse** (idempotent) and **pp_graph_get_storage_info** for base/reverse components in both release and metrics builds. Update headers, ABI, ctypes, and docs; a second prepare must not increase retained bytes.

### 8.2. Anytime and resource budgets

Anytime currently restarts weighted A* at each weight, sums iters/nodes, and returns the best final path. time_to_first_path is not currently in the output. A separate ticket adds pass index/weight/start/end/counters, incumbent cost/improvement events, and a first-valid-path timestamp. The experimental schedule [2.0,1.5,1.25,1.0] belongs in variant_id.

Quality-versus-expanded requires one total expansion budget per query, passing the remaining budget to each pass; the old max_expansions is per-pass. Add a separate total-budget parameter/contract and verify that it is not reset. Path availability and termination cause are distinct: an incumbent may exist when the search stops due to a budget.

Quality-versus-wall-time requires a cooperative native deadline so the search can return an incumbent. A watchdog kill only produces a timeout and cannot return the incumbent; publish this curve only after cooperative return is implemented. One A* expansion and one JPS scan do not have equal cost.

## 9. Complexity, scaling, and ablation

Let N=allocated slots, M=edges_examined, P=frontier_pushes, D=solution steps. Analyze the current implementation, with O(1) heuristic lookup and no reopening:

- BFS/DFS query: O(N+M+D), state/frontier O(N+frontier_peak).
- Best-first lazy heap: O(N+M+P*log(max(2,P))+D); each pass has P<=E+1, giving the upper bound O(N+E*log(max(2,E))) under these assumptions.
- State is O(N); heap memory depends on peak/capacity and may contain duplicates; input CSR is separately O(N+E).
- Bidirectional search adds two states/frontiers; reverse preparation is O(N+E). Bidirectional A* also performs full consistency validation on every query in O(N+E).
- Anytime K passes add initialization/search work per pass; peak memory follows live lifetimes and is not K times the state peak.

Update the bound and its assumptions for every change to reopening, heap, heuristic, representation, or workspace reuse.

| Initial sweep | Design |
| --- | --- |
| Size | L=64,128,256,512,1024; random density=0.20; 5 generator seeds; 20 queries/map in fixed normalized-displacement bins. Keep movement, CSR layout, and h-mode fixed. |
| Density | L=512, density=0.10,0.20,0.30,0.40; 5 seeds; record connectivity and unreachable cases. |
| Heuristic | Same map/query, h=alpha*octile with alpha=0,0.25,0.5,0.75,1; consistent. Give weighted-A* weight sweeps a separate name. |
| Topology | Room/maze corridor/opening width=1,2,4,8 at the same L; record density/connectivity changes alongside the parameter. |

Plot work/memory against N/V_free/E; log-log slopes with fit range and CI show empirical trends and do not prove Big-O. [Sturtevant 2012](https://www.cs.du.edu/~sturtevant/papers/benchmarks.pdf) shows that scaling a map changes its spatial properties; record topology covariates instead of assuming resizing preserves everything.

Initial ablations: fresh/reused graph; precomputed/lazy h if the backend supports it; release/metrics overhead; forward/bidirectional. Change one factor per ablation and report work/time/memory/quality. ns/expansion is an aggregate ratio, not the causal cost of one expansion, because it includes initialization/validation and other work.

Solve the A/B break-even point from P_B+Q*q_B <= P_A+Q*q_A. When P_B>P_A and q_B<q_A: Q*=ceil((P_B-P_A)/(q_A-q_B)); classify other cases separately. Estimate q from the same query distribution, include baseline preparation/load costs, and account for storage budget separately.

## 10. Report v2, outputs, and CLI

The v2 builder lives in `scripts/shortest_path_benchmark/contract.py`, reuses provenance/environment helpers from `benchmark_contract.py`, and adds hashes for the metrics binary. Keep the current `create_report` and `SCHEMA_VERSION` v1 contract unchanged. Outputs:

~~~text
benchmark-results/<campaign>/
  manifest.json
  schedule_<pass>_<scope>_<graph_state>.json
  <pass>_<scope>_<graph_state>.json
  oracle.jsonl
  runs.jsonl
  run_summary_<pass>_<scope>_<graph_state>.json
  summary.json
  report.md
  plots/*.svg
~~~

Each row contains run/workload/variant/campaign IDs, phase/repeat/order/scope/graph_state/budget, and input/outcome/work/memory/timing/capabilities/provenance. Preserve exact integers >2^53; unavailable/nonfinite values are null + reason, never NaN/Infinity.

JSONL records are complete and checkpointed/flushed; resume validates manifest/binary/protocol hashes. Mark an incomplete final line for crash recovery and run only unfinished keys. Retain terminal timeout/error records. Do not silently change primary coverage by keeping only successful retries. Write summary snapshots atomically.

The CLI exits nonzero on correctness/ABI/manifest gate failures, crashes, or missing expected observations. A valid approximate path with a positive gap is a correct outcome if declared as such. A missing required metric produces incomplete_measurements.

Full/scaling campaigns must report per-map/family/difficulty results, paired work ratios, coverage-versus-budget, and Pareto time-memory-quality with workload/protocol; annotate coverage and quality. The current MVP report includes aggregate latency/work/memory and paired statistics; full strata, budget frontier, and Pareto reporting remain in SP-09/SP-10. EBF, transit-node count, and estimated-diameter/map-dimension proxy are optional descriptors, not required MVP metrics.

Implemented CLI:

~~~text
python scripts/benchmark_shortest_path.py prepare --profile pilot --manifest <path>
python scripts/benchmark_shortest_path.py validate --manifest <path>
python scripts/benchmark_shortest_path.py run --manifest <path> --pass latency --scope public_api --graph-state reused_graph
python scripts/benchmark_shortest_path.py run --manifest <path> --pass work
python scripts/benchmark_shortest_path.py run --manifest <path> --pass memory
python scripts/benchmark_shortest_path.py analyze --campaign <directory>
~~~

Latency/work/memory passes have separate provenance. The analyzer checks compatibility keys; combining the three passes does not make them simultaneous measurements. The Markdown report shows latency median/P95 and paired speedup; work counters and allocator memory median/P95; fresh-worker RSS plus occupancy/CSR/retained-graph memory. `summary.json` retains full vectors with count, median, P95, min, max, IQR, paired ratios, and bootstrap intervals. RSS covers the process lifetime, including Python, NumPy, reading the map, and graph creation; do not interpret it as memory used by the search query alone.

## 11. Backlog, dependencies, and acceptance gates

Keep these acceptance gates to distinguish the MVP from extension tracks. Run Graphify update after code changes; keep generated output out of Git.

| Ticket | Planned deliverable / files | Dependency | Acceptance |
| --- | --- | --- | --- |
| SP-01 | scripts/shortest_path_benchmark/contract.py, profiles/pilot.json, profiles/scaling.json; schema dictionary and contract docs | Plan | V1 structure unchanged; v2 round-trips integers >2^53; missing/zero/unsupported and denominators are correct. |
| SP-02 | workloads.py: parser, no-corner CSR, vectorized heuristic adapter, manifest | SP-01 | Validate version/symbol/dimension/path/bounds; corner fixtures have correct edges; pilot IDs/hashes are reproducible. |
| SP-03 | reference.py: independent oracle, validator, and cache | SP-02 | Known costs/start=goal/disconnected/tied paths/declared-cost mismatch; retain scenario discrepancies. |
| SP-04 | search_metrics.h, search_engine.cpp/.h, abi_version.h, _ffi.py, setup.py, docs/native_abi.md; metric hooks/build/loader and graph APIs | SP-03 | Release/metrics parity for every MVP variant on fixtures/subset; hand-counted counters; correct ABI/free/ownership; idempotent reverse preparation and separate cache. |
| SP-05 | Allocation tracking + memory observations | SP-04 | Correct slots/dtype/capacity; measure heap duplicates/reallocation overlap; total peak from live sums; counters available for failed/unreachable calls. |
| SP-06 | runner.py + benchmark_shortest_path.py: workers, scopes, schedule, watchdog, JSONL/resume | SP-01..05 | Replay schedule; sequential queries; timeout at correct stage; correct graph state; retain crashes/missing records; distinguish unique-query and repeat denominators. |
| SP-07 | analysis.py: paired statistics, cluster bootstrap, weights, report/plots | SP-06 | Fixtures verify median/ratio/censoring/unmatched/zero-cost; reproducible CI seed; plots show units/counts. |
| SP-08 | End-to-end pilot, build/environment evidence, and reproducibility docs | SP-07 | Work/correctness on up to 1,800 queries, latency/RSS subset up to 360; no unexplained validity/optimality mismatch; metrics parity; required fields present in each measurement cohort. Gate before full campaign. |
| SP-09 | Anytime pass observations, global budget, cooperative incumbent return | SP-08 | Correct work sums; total budget is not reset; verify first/last incumbent; correct termination and path availability. |
| SP-10 | Full/scaling/ablation campaigns and frontier reports | SP-08; SP-09 for anytime | Freeze cohort/config; complete expected records; separate evidence for bounds and slopes; include every failure/limitation in the report. |

### Implementation status, 2026-10-06

| Ticket | Status |
| --- | --- |
| SP-01–SP-06 | Implemented; contract, MovingAI parser/oracle, native metrics/allocation hooks, and runner have tests. |
| SP-07 | MVP report/plot implemented: latency, work counters, allocator memory, process RSS, quality, paired ratios, and bootstrap. Full per-map/difficulty strata, coverage-versus-budget, and Pareto frontier remain in SP-09/SP-10. |
| SP-08 | End-to-end pilot passed: 1,750 workloads × 8 variants for work (14,000 observations); 360 workloads × 8 variants for latency (20,160 measured + 5,760 warmups) and memory (2,880 observations). All 42,800 observations were `ok`; there were no invalid paths, missing required metrics, or unexplained oracle discrepancies. Output in `benchmark-results/` is gitignored and must be regenerated using the reproduction guide. |
| SP-09 | Not implemented: anytime pass observations, total resource budget, and incumbent events. |
| SP-10 | Partial: scaling profile/generators, log-log fit, and ablation identity helpers exist; full/scaling/ablation campaigns and Pareto-frontier evidence have not been run/completed. |

SP-01→SP-08 comprise the MVP, and the pilot passed the correctness gates. SP-09/SP-10 remain follow-up work; the existence of profiles or sweep helpers does not mean those items are complete. Existing tests include `test_movingai_workloads.py`, `test_shortest_path_reference.py`, `test_native_metrics.py`, `test_shortest_path_benchmark_contract.py`, `test_shortest_path_benchmark_runner.py`, `test_shortest_path_benchmark_analysis.py`, and `test_shortest_path_scaling.py`. Use the [native graph tests](../tests/test_native_graph_search.py) and [trace parity tests](../tests/test_native_trace.py) for regression coverage.

Native/API validation requires release/metrics/trace builds, focused tests, ruff/pyright, and the non-slow suite from the Makefile when in scope; expand to full/slow tests based on failures/risk. This pilot ran the focused suite and Ruff; the test files above are implemented tests, not planned tests.

## 12. Retained extension tracks

| Track | Additional metrics | Requirement |
| --- | --- | --- |
| JPS/grid pruning | Scanned cells/blocks, repeated scans, jump points, forced-neighbor checks, prune reason | An actual planner with the same movement/cost model; expansion alone does not represent scan work. |
| Any-angle | LOS calls, primitive cells/segments checked, geometric cost/validity | Use an oracle for the correct objective; octile optimum is not an any-angle optimum. |
| Sampling | Attempted/accepted/rejected samples; NN queries/distance evaluations; collision calls/steps; rewire attempts/success; tree/index peak; quality/coverage by budget and RNG seeds | Separate continuous-space workload; reuse existing sample_count/motion_checks/rewires; do not combine nodes with expansions. |
| Incremental/real-time | First prefix, maximum segment latency, repair/update work, retained state, amortized sequence cost | The API must return an actual prefix/update and use a fixed change trace. |
| Hardware profiling | Instructions/cycles/cache/branch misses/allocations | Run separately when the platform supports it; unavailable=null and machine dependence must be recorded. |

[GPPC 2014](https://webdocs.cs.ualberta.ca/~nathanst/papers/GPPC-2014.pdf) evaluates preprocessing, memory, quality, latency, and Pareto trade-offs. This plan adds implementation-level work definitions, memory lifetimes, and protocol so the mechanisms can be explained.

## 13. Completion criteria

The MVP is complete when it has a validated parser/profile, independent oracle, work metrics with defined semantics/coverage, correct release scopes, properly labeled native memory and RSS, a reproducible v2 report, and a pilot that passes correctness gates.

Research results additionally require frozen full/scaling cohorts, paired distributions/sample counts, coverage/failures, data/build provenance, and ablations for the claimed mechanisms. Clearly label each item proposed/implemented/measured; runtime numbers or a standalone dashboard do not satisfy these gates.
