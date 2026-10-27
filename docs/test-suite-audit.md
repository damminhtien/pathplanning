# Test suite audit — 2026-10-27

## Result and selection

Baseline: `8463b69e452fb234ccd054f75774e7b59c79a8ed`.
The maintained suite decreases from **357 to 107 collected pytest cases**:
**250 fewer cases (70.03%)**, retaining 29.97%. Test functions decrease from
263 to 88 and test files from 62 to 37. The 107-case allowance retains more
independent correctness, planner, native-boundary and stateful behavior than
the preferred 71-case target. This audit does not establish that 71 is
impossible, that 107 is minimal, or that defect-detection power is unchanged.

[The CSV ranks every original case](test-suite-ranking.csv): **107 retained,
3 assertion merges, 247 removed**. No skip/xfail or alternate tier hides
removed cases. Parametrized variants count separately. Existing seeded oracle
campaigns remain unchanged; new loops validate one returned path's steps,
never hide independent scenarios. All **37 canonical registry callables**
executed in the reduced audit and have behavioral witnesses below.

Production sources, public API, registry, native ABI, dependencies, Makefile
and CI are unchanged. This is local evidence, not a fresh remote CI or
wheel-matrix result.

## Ranking method

P0 protects critical correctness/oracles and native safety; P1 primary planner
behavior; P2 additional contracts; P3 supporting tools; P4 repeated structural
or weak smoke coverage. Classification and assertion quality are engineering
judgments, exposed alongside original assertions and decision reasons in the
CSV. The retained mix is **26 P0, 26 P1, 50 P2 and 5 P3** cases. Planner coverage
is a constraint across ranks, so P1's case count is not the planner count.

The score sums these disclosed components:

- Impact /35: P0=35, P1=30, P2=25, P3=15, P4=7.
- Oracle /25: independent oracle=25; production/diagnostic parity=21;
  behavioral assertions=20; expected error=17; structural/import contract=10;
  execution only=4; weak smoke=3.
- Incremental /20: `min(20, round(4*log2(1+exclusive_lines+exclusive_branches)))`.
- Breadth /10: registry/library/configuration-wide=10; native or independent
  oracle=8; otherwise=5.
- Cost /10, using instrumented baseline duration: <0.05s=10; <0.2s=8;
  <1s=6; otherwise=2. These durations are not runtime benchmarks.

Required behavior takes precedence over score. Related cases in the CSV are
navigation aids, not equivalence claims. Coverage overlap cannot establish
assertion strength; import-time lines can be shared and a static guard can
have zero executed production lines while remaining useful.

Retained scenarios gain endpoint, adjacency, cost and collision assertions;
MAPF gains explicit vertex/edge-swap checks. RRT/RRT* collision/goal/cost
assertions move into their fixed-seed scenarios, and release/metrics parity
moves into the existing reopen scenario. Distinct retired input variants are
not claimed to be preserved. Public PRM/Lazy PRM/EIRM entrypoints and stateful
roadmap reuse both retain behavioral checks.

An audit attempt exposed an existing DynamicRRT callback-test flake: an
entropy-seeded RNG could select a goal/waypoint without calling `sample_free`.
Both measurement suites normalize only that baseline fixture to seed 4 with
goal/waypoint probabilities zero. Node IDs and production code are unchanged.
Final measurements use green runs and fresh profiles; failed exploratory
runs are excluded.

## Coverage protocol

Host: Apple M2 Max, macOS 27.0.1 arm64, CPython 3.14.8, NumPy 2.5.3,
pytest 9.1.1, coverage.py 7.17.0, Apple clang 21. Both sides use identical
production source bytes and source denominators. The installed pytest is
newer than the declared `pytest>=8,<9` range; all **107 cases also pass with
pytest 8.4.2** installed outside the project environment.

Python measurement uses `COVERAGE_CORE=ctrace`, branch tracing, separate
nodeid setup/call/teardown contexts, and a temporary pytest plugin. Parameter
variants have separate contexts. Subprocesses inherit `COVERAGE_PROCESS_START`,
a temporary `sitecustomize` calling `coverage.process_startup()`, parallel data
files and the active test context. Multiprocessing tracing is enabled.
Files are combined after process exit. Isolated/external tools not inheriting
that startup hook are outside the Python measure.

The complete `pathplanning` and `scripts` Python source roots include unexecuted
files; no new exclusions or omissions are added. CSV line/branch counts cover
that case's Python setup/call/teardown/subprocess execution. Exclusive means
exclusive among attributed test contexts; collection imports are separate.
Cases executing no production statements have zero counts, not missing rows.
The CSV records baseline assertions and coverage; strengthened assertions are
described above. Python `native` coverage is FFI code, not C/C++ coverage.

### Python results

| Subsystem | Lines before | Lines after | Branches before | Branches after |
| --- | ---: | ---: | ---: | ---: |
| `pathplanning/core` | 508/603 (84.2%) | 480/603 (79.6%) | 153/244 (62.7%) | 129/244 (52.9%) |
| `pathplanning/data_structures` | 15/41 (36.6%) | 15/41 (36.6%) | 0/8 (0.0%) | 0/8 (0.0%) |
| `pathplanning/geometry` | 0/826 (0.0%) | 0/826 (0.0%) | 0/250 (0.0%) | 0/250 (0.0%) |
| `pathplanning/native` | 1282/1486 (86.3%) | 1264/1486 (85.1%) | 291/438 (66.4%) | 279/438 (63.7%) |
| `pathplanning/nn` | 41/161 (25.5%) | 41/161 (25.5%) | 0/46 (0.0%) | 0/46 (0.0%) |
| `pathplanning/planners` | 2141/2585 (82.8%) | 2035/2585 (78.7%) | 517/800 (64.6%) | 450/800 (56.2%) |
| `pathplanning/root` | 170/192 (88.5%) | 164/192 (85.4%) | 38/58 (65.5%) | 34/58 (58.6%) |
| `pathplanning/spaces` | 930/1242 (74.9%) | 756/1242 (60.9%) | 243/450 (54.0%) | 188/450 (41.8%) |
| `pathplanning/utils` | 71/114 (62.3%) | 0/114 (0.0%) | 8/28 (28.6%) | 0/28 (0.0%) |
| `pathplanning/viz` | 495/582 (85.1%) | 419/582 (72.0%) | 139/204 (68.1%) | 101/204 (49.5%) |
| `scripts` | 3089/6996 (44.2%) | 1224/6996 (17.5%) | 984/2528 (38.9%) | 279/2528 (11.0%) |

### Native C/C++ results

The five libraries were built once in the temporary snapshot with CFLAGS and
CXXFLAGS `-fprofile-instr-generate -fcoverage-mapping` and LDFLAGS
`-fprofile-instr-generate`, retaining setup.py's C11/C++17 and O3 flags. Both
suites reuse the same binaries. `LLVM_PROFILE_FILE=%p-%m.profraw` separates
process/module profiles. Profiles are classified by ABI probe, merged **per
library**, and exported with the matching binary using `llvm-cov export`.
Totals include only `pathplanning/native` source/header records, excluding
system/STL headers. An exploratory merge across production/diagnostic variants
produced function-profile mismatches; that aggregation is not used here.

| Library | Lines before | Lines after | Branches before | Branches after |
| --- | ---: | ---: | ---: | ---: |
| `_search_engine` | 5048/6575 (76.8%) | 4863/6575 (74.0%) | 2088/3206 (65.1%) | 1983/3206 (61.9%) |
| `_continuous_engine` | 3601/4369 (82.4%) | 3530/4369 (80.8%) | 2229/3370 (66.1%) | 2177/3370 (64.6%) |
| `_search_trace_engine` | 4752/6523 (72.8%) | 2568/6523 (39.4%) | 1791/3188 (56.2%) | 942/3188 (29.5%) |
| `_continuous_trace_engine` | 3488/4731 (73.7%) | 2450/4731 (51.8%) | 2010/3528 (57.0%) | 1368/3528 (38.8%) |
| `_search_metrics_engine` | 1055/1527 (69.1%) | 487/1527 (31.9%) | 435/813 (53.5%) | 198/813 (24.4%) |

The first two rows are production; the remaining rows are diagnostics. Do not
pool their percentages: compiled variants share source but differ in branches.
The reduced suite retains **96.3% of already-covered production search lines**
and **98.0% of already-covered production continuous lines**. These are
retention ratios, not overall coverage or defect-detection percentages.
Native measurements are suite-level; CSV per-case coverage is Python only.
No mutation-testing percentage is claimed.

## Behavioral protection matrix

Each witness was observed executing the canonical callable and reviewed for
behavioral assertions. Registry import and support-document consistency checks
supplement these cases and do not substitute for them.

| Kind | Planner | Retained behavioral witness |
| --- | --- | --- |
| temporal | `sipp` | `tests/test_sipp.py::test_sipp_matches_small_time_expanded_oracle_and_traces` |
| temporal | `bounded_suboptimal_sipp` | `tests/test_sipp.py::test_bounded_suboptimal_sipp_uses_focal_hops_and_obeys_weight` |
| temporal | `kinodynamic_sipp` | `tests/test_sipp.py::test_kinodynamic_sipp_projects_waits_from_stopped_configurations` |
| multi_agent | `eecbs` | `tests/test_eecbs.py::test_eecbs_matches_joint_oracle_bound_and_traces` |
| multi_agent | `lacam_star` | `tests/test_lacam_star.py::test_lacam_star_matches_joint_oracle_and_records_trace` |
| discrete | `bfs` | `tests/test_native_trace.py::test_search_trace_matches_production[bfs]` |
| discrete | `dfs` | `tests/test_native_trace.py::test_search_trace_matches_production[dfs]` |
| discrete | `greedy_best_first` | `tests/test_native_trace.py::test_search_trace_matches_production[greedy_best_first]` |
| discrete | `astar` | `tests/test_native_trace.py::test_search_trace_matches_production[astar]` |
| discrete | `bidirectional_dijkstra` | `tests/test_native_trace.py::test_search_trace_matches_production[bidirectional_dijkstra]` |
| discrete | `bidirectional_astar` | `tests/test_native_trace.py::test_search_trace_matches_production[bidirectional_astar]` |
| discrete | `dijkstra` | `tests/test_native_trace.py::test_search_trace_matches_production[dijkstra]` |
| discrete | `dstar_lite` | `tests/test_dstar_lite.py::test_dstar_lite_grid_adapter_keeps_blocked_edges_updateable` |
| discrete | `weighted_astar` | `tests/test_native_trace.py::test_search_trace_matches_production[weighted_astar]` |
| discrete | `reexp_astar` | `tests/test_reexp_astar.py::test_reexp_astar_reopens_closed_node_at_zero_threshold` |
| discrete | `anytime_astar` | `tests/test_native_trace.py::test_search_trace_matches_production[anytime_astar]` |
| discrete | `jps` | `tests/test_jps.py::test_jps_matches_strict_dijkstra_on_seeded_random_maps` |
| discrete | `jpsw` | `tests/test_jpsw.py::test_jpsw_matches_dijkstra_on_seeded_weighted_grids` |
| discrete | `theta_star` | `tests/test_theta_star.py::test_theta_star_integrates_weighted_terrain_along_segments` |
| discrete | `lazy_theta_star` | `tests/test_lazy_theta_star.py::test_lazy_theta_star_integrates_weighted_terrain` |
| continuous | `rrt` | `tests/test_native_trace.py::test_sampling_trace_matches_production[rrt]` |
| continuous | `rrt_star` | `tests/test_problem_configuration_regressions.py::test_rrt_star_uses_problem_objective_and_problem_params` |
| continuous | `informed_rrt_star` | `tests/test_native_trace.py::test_sampling_trace_matches_production[informed_rrt_star]` |
| continuous | `fmt_star` | `tests/test_native_trace.py::test_sampling_trace_matches_production[fmt_star]` |
| continuous | `prm_star` | `tests/test_prm_star.py::test_prm_star_registry_entry_uses_native_roadmap` |
| continuous | `lazy_prm` | `tests/test_lazy_prm.py::test_lazy_prm_is_registered_for_continuous_problems` |
| continuous | `ait_star` | `tests/test_eit_star.py::test_eit_star_finds_a_valid_path_around_obstacle` |
| continuous | `eit_star` | `tests/test_eit_star.py::test_eit_star_finds_a_valid_path_around_obstacle` |
| continuous | `eirm_star` | `tests/test_eirm_star.py::test_eirm_star_registry_entry_uses_roadmap_params` |
| continuous | `fcit_star` | `tests/test_fcit_star.py::test_fcit_star_searches_complete_graph_and_batches_motion_checks` |
| continuous | `rit_star` | `tests/test_rit_star.py::test_rit_star_uses_constant_anisotropic_metric_for_path_cost` |
| continuous | `jit_star` | `tests/test_jit_star.py::test_jit_star_uses_jacobian_cost_and_collision_guidance` |
| continuous | `hybrid_astar` | `tests/test_hybrid_astar.py::test_hybrid_astar_checks_the_full_footprint_around_an_obstacle` |
| continuous | `state_lattice` | `tests/test_state_lattice.py::test_state_lattice_checks_each_primitive_through_occupied_cells` |
| continuous | `bit_star` | `tests/test_native_trace.py::test_sampling_trace_matches_production[bit_star]` |
| continuous | `abit_star` | `tests/test_native_trace.py::test_sampling_trace_matches_production[abit_star]` |
| continuous | `rrt_connect` | `tests/test_native_trace.py::test_sampling_trace_matches_production[rrt_connect]` |

Additional stateful checks protect D* Lite repair/start movement; PRM
world-version invalidation; PRM/Lazy PRM/EIRM reuse; and DynamicRRT seeded
target selection, callback opt-in, pruning and regrowth. Three native obstacle
cases retain sparse-sample collision regressions for spheres, AABBs and OBBs.
Temporal tests retain waiting, half-open intervals, bounded search and
kinodynamic timing. MAPF retains independent joint-state oracles and conflicts.

## Accepted losses

- Specialized planner trace parity is narrower. Metrics loses its full
  ten-algorithm parity matrix, reverse-storage, validation-work and
  production-symbol checks; stale/reopen counters and one release-parity
  fixture remain. Diagnostic coverage falls substantially, as shown above.
- Parameter rejection matrices, protocol/signature/source scanning, historic
  legacy/naming guards, global-RNG monkeypatching, helper fallback/parent-chain
  errors and nearest-index parity lose dedicated coverage. Fixed-seed behavior
  does not replace every removed RNG guard.
- Dataset characterization/discovery/selection, parser/cohort variants,
  installer happy-path/retry/cache/resource limits, and most benchmark
  scheduling/statistics/provenance cases are retired. Independent path cost,
  reference path validation, scenario-vs-independent-oracle validation,
  timeout/coverage denominator accounting and installer ZIP traversal rejection
  remain; they do not replace all removed ingestion/provenance contracts.
- Visualization retains backward seek and viewer cleanup. Separate export,
  camera and truncated-display assertions are retired. Standalone `utils`
  execution falls to zero; geometry was already zero. Planner tests are not
  claimed to cover those utilities.

The lower supporting-tool and diagnostic coverage is an explicit cost of this
reduction. A future change in those areas requires appropriate regressions;
107 is a snapshot budget, not a permanent ceiling.

## Validation and reproduction

Fresh native build, all/unit/slow runs, Ruff check/format, Pyright,
generated-registry consistency, ranking integrity and production-byte equality
pass locally: **107 total, 105 unit, 2 slow**. The final supported-pytest check
also passes all 107 cases using pytest 8.4.2. Graphify rebuilt 4,398 nodes and
12,639 edges. It reports existing parser limitations in four native headers;
native build/test results are the validation evidence for those files.
Generated graph outputs remain ignored. No remote CI run is claimed.

Three sequential uninstrumented pytest runs on this host use warm dependency
and build caches and `MPLBACKEND=Agg`. They exclude build time and coverage
instrumentation. Baseline runs precede candidate runs rather than alternating;
these are local suite measurements, not planner-performance benchmarks:

- baseline: 18.37s, 17.86s, 17.78s; median **17.86s**.
- candidate: 6.06s, 6.96s, 6.53s; median **6.53s**.

Median suite time decreases by **63.4%** (2.74x on this host).

Run maintained gates using the project environment:

```sh
make test-unit NATIVE_BUILD_DIR=/tmp/pathplanning-audit-clean-build
make test-slow NATIVE_BUILD_DIR=/tmp/pathplanning-audit-clean-build
MPLBACKEND=Agg python -m pytest -q
ruff check pathplanning tests scripts examples
ruff format --check pathplanning tests scripts examples
pyright
python scripts/generate_supported_algorithms.py --check
git diff --check
```

To reproduce a comparison, export the baseline commit to a temporary checkout,
normalize only the documented DynamicRRT fixture, and use the same interpreter
and coverage settings on both snapshots. Per-case Python coverage requires the
context/subprocess hooks described above; a plain coverage run is not an exact
replacement. For native coverage, build the five libraries with the documented
flags, run pytest with separate profile output directories, and identify each
raw profile by its ABI probe using `llvm-profdata show --all-functions`.
Merge raw profiles for each library with `llvm-profdata merge -sparse`, then
run `llvm-cov export LIBRARY -instr-profile=PROFILE` and filter to project native
sources. On macOS these tools are available through `xcrun`.

Do not run a multiprocessing audit from stdin, reuse stale native binaries,
merge failed attempts, or pool production and diagnostic function profiles.
The exact temporary audit harness, logs, per-case contexts, native exports and
source hash manifest remain locally in `/tmp/pathplanning-audit-tools` and
`/tmp/pathplanning-audit-evidence`. They are temporary, uncommitted artifacts;
the CSV and this report are the durable review record. Future changes require
fresh measurements.

Production-source SHA-256 manifest fingerprint:
`bcb899717bb2de5015fa7dcaba1bb9ae4cd18f0cb93d78ff2a1ee67ae939e7c1`.

The manifest hashes tracked `pathplanning`, `scripts`, `setup.py` and
`pyproject.toml` files; their bytes were compared against the frozen baseline.
Native binaries are generated artifacts.
