# Shortest-path benchmark input and record contract

The implementation follows the protocol in [the evaluation plan](shortest_path_evaluation_plan.md).
This is a separate `pathplanning_shortest_path_v2` report family. Existing
`pathplanning_benchmark_v1` reports and planner return values keep their current
meaning.

The source inventory, benchmark coverage snapshot, and input illustrations are
in the [dataset characterization](benchmarks/dataset_characterization.md).
Execution commands and current campaign counts are in the
[reproduction guide](shortest_path_benchmark_reproduction.md).

## MovingAI input profile

`land_octile_v1` accepts ASCII `type octile` maps with exact declared dimensions.
`.` / `G` / `S` are passable; `@` / `O` / `T` / `W` are blocked. Unknown symbols
fail input validation. Scenario headers `version 1` and `version 1.0` are accepted
with exactly nine fields per row. The collection's version and the scenario-file
version are distinct. Map paths must resolve inside the declared dataset root.
Scaled scenario dimensions are reported as `unsupported_scaled_scenario`.

Cell ID is `y * width + x`, with `(0,0)` at the upper left. CSR retains every
cell slot, including blocked rows. Each edge costs 1 or `sqrt(2)`; a diagonal
requires both adjacent cardinal cells to be passable. The eight directions follow
the repository grid order. `node_slots`, `free_nodes`, and `directed_edges` are
separate input descriptors. The Python reference computes Dijkstra distance
directly from occupancy and does not use the CSR or native search code.

## Workload and result identity

`workload_id` hashes map content, movement profile, start and goal. It does not
include algorithm, run order, host, or recorded scenario optimum. `variant_id`
must hash algorithm, parameter values, heuristic mode, and tie policy.
`experiment_id` additionally identifies source, binaries, host, and protocol.
Pairing compares workload and compatible protocol; it does not require equal
experiment IDs across builds.

Source-scenario cases use `query_kind: source_scenario`. Seeded negative cases
use `query_kind: unreachable`, identify their free-space component labels, and
belong to a separate `unreachable` cohort. For strict octile movement with no
corner cutting, four-neighbor component labels exactly characterize
reachability. Negative cases must have an independent oracle result of
`reference_reachable: false`; they do not have a scenario optimum and are never
implicitly pooled into the source-scenario cohort.

The `coverage` profile scans the complete MovingAI land scenario collection,
then selects one hash-chosen unique query per map and normalized-displacement
bin. Its `work` cohort reaches every map with source scenarios; its `latency`
cohort uses one selected query per map, and its `memory` cohort uses up to three
map-size quantiles per family with one query per map. This is a stratified
coverage sample, not a run of every source scenario row. Synthetic scaling
inputs use `query_kind: synthetic_grid`; their independent Dijkstra reference
defines reachability and cost because they have no source optimum.

The grid-specialist runner reuses the validated coverage latency query for
JPS, D* Lite, Theta*, and Lazy Theta*. JPS and D* Lite are checked against
`land_octile_v1`. Theta* variants use square-obstacle visibility and are
reported as a separate any-angle cohort with collision validation but no
octile optimality claim. JPSW is unavailable until a source terrain cost table
is provided.

Each run record has `schema_version`, run/workload/variant/campaign IDs, pass,
phase, repeat, order, scope, graph state, and seven namespaces: `input`,
`outcome`, `work`, `memory`, `timing`, `capabilities`, `provenance`. Work and byte
counters are exact JSON integers, including values above `2^53`. A measured
zero is `0`. An unavailable measurement is `null` with an explicit reason;
nonfinite numbers are rejected. Optional metric objects can use
`{"value": null, "reason": "unsupported"}`. A missing field is not a measured
zero. Ratios with a zero denominator are `null` and carry a reason.

`runs.jsonl` is appended one complete line per invocation and synced outside
the measurement bracket. On resume, only an incomplete last line may be
discarded; completed errors and timeouts remain observations. `summary.json`
is replaced atomically. Each pass records its own measurement scope, graph
state, binary hashes, and provenance; counters from a metrics build must not
be presented as simultaneous release latency measurements.

## Correctness and denominators

The oracle returns `None` for unreachable goals and exact `0` when start equals
goal. A candidate path is checked for endpoints, legal steps, obstacles, corner
cutting, and independently reconstructed cost. Declared cost and optimality are
separate checks. Candidate-to-oracle comparison uses absolute tolerance `1e-8`
and relative tolerance `1e-10`. Scenario optimum comparison also allows one unit
of its last printed decimal place. The selected MovingAI archives serialize
diagonal cost as `1.414213562`; when that differs from exact `sqrt(2)`, validation
infers the unique cardinal/diagonal step counts from the oracle cost and checks
the source string against that serialized value within half a printed unit.
This format allowance never applies to candidate-to-oracle checks. Scenario
disagreement is an input/reference discrepancy to resolve before measuring
candidates.

Coverage counts unique oracle-solvable workloads selected before candidate
execution. Repeated invocations, warmups, crashes, and timeouts remain in their
own denominators. Unreachable decision accuracy has a separate denominator
and a separate cohort selection target; it must be reported alongside, not
substituted for, source-scenario reachability coverage.

## Other installed formats

The format runner at scripts/benchmark_format_cohorts.py keeps each source
format in its own cohort. DIMACS .gr files retain their directed weighted
parallel arcs as source data; the search graph uses the minimum-cost arc for
each ordered endpoint pair, preserving shortest-path costs and supporting
incremental D* Lite. It uses one seeded reachable pair from the largest
strongly connected component and reports an independent SciPy Dijkstra
oracle. Algorithms receive a zero heuristic because road-coordinate units
and edge-cost units are not assumed interchangeable. Graphs above the
configured node or arc limit remain explicitly resource_limited.

Voxel maps use source scenario endpoints and a strict 26-neighbor Euclidean
graph. Diagonal moves require every proper side cell to be free. Byte-identical
MovingAI/Monash mirrors run once under the canonical source and remain marked
as mirror duplicates in the inventory. BARN worlds form a separate derived
point-robot XY cohort: collision cylinders are checked geometrically, a
collision-free grid path certifies each selected query is reachable, and the
provided .npy paths are retained as provenance but not interpreted as world
coordinates. BARN uses exact point goals and native sampled collision checks
with each source circle inflated by half the 0.01 m check step (plus a 1 nm
numeric guard). This guarantees that an edge intersecting a source circle is
rejected by at least one sampled check; returned paths are validated exactly
against the source circles. The margin is recorded on every observation. JIT*
is excluded because the derived point-robot space has no robot Jacobian for its
manipulability score. Voxel records preserve the source scenario line, reported
optimum, and difficulty and report whether the source optimum agrees with the
independent oracle for the strict movement profile. These continuous results
have no independent optimality oracle.

The format runner records source-asset hashes and separate inventory statuses
for completed, resource-limited, mirrored, and invalid inputs. OMPL.app
configuration resources are installed but are not planner measurements unless
the matching OMPL.app robot-mesh collision checker is available. Missing
terrain cost tables, temporal/MAPF data, and kinodynamic robot/control models
remain unsupported rather than being mapped onto a different cohort.
