# Shortest-path benchmark input and record contract

The implementation follows the protocol in [the evaluation plan](shortest_path_evaluation_plan.md).
This is a separate `pathplanning_shortest_path_v2` report family. Existing
`pathplanning_benchmark_v1` reports and planner return values keep their current
meaning.

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
own denominators. Unreachable decision accuracy has a separate denominator.
