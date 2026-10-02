# Environment Architecture

## Why environments are abstract

Planners must depend on behavior contracts, not concrete map/world classes.

- Reuse: one planner works with many world representations.
- Extensibility: users can plug in custom environments without changing planner code.
- Testability: fake/minimal environments can be injected in unit tests.
- Maintainability: no duplicated `env.py`/`env_3d.py` logic inside algorithm folders.
- Import safety: planner modules stay lightweight and avoid geometry/rendering dependencies.

## Core interfaces (exact contract names)

Defined in `pathplanning/core/contracts.py` and typed with `S`, `N`, `RNG` from `pathplanning/core/types.py`.

### Discrete / graph-search

- `DiscreteGraph[N]`
  - `neighbors(n: N) -> Iterable[N]`
  - `edge_cost(a: N, b: N) -> Float`
- Optional extensions
  - `HeuristicDiscreteGraph[N]`
    - `heuristic(n: N, goal: N) -> Float`
  - `ValidatingDiscreteGraph[N]`
    - `is_valid_node(n: N) -> bool`
- Goal abstraction
  - `GoalTest[N]`
    - `is_goal(n: N) -> bool`
  - `ExactGoalTest[N]` (adapter for exact goal node)
- Problem wrapper
  - `DiscreteProblem[N]`
    - `graph: DiscreteGraph[N]`
    - `start: N`
    - `goal: N | GoalTest[N]`
    - `params: Mapping[str, object] | None`

### Continuous / sampling

- `ContinuousSpace[S]`
  - `sample_free(rng: RNG) -> S`
  - `is_state_valid(x: S) -> bool`
  - `is_motion_valid(a: S, b: S) -> bool`
  - `distance(a: S, b: S) -> Float`
  - `steer(a: S, b: S, step_size: Float) -> S`
- Optional extensions
  - `InterpolatingContinuousSpace[S]`
    - `interpolate(a: S, b: S, t: Float) -> S`
  - `ContinuousSpaceMetadata`
    - `bounds`
    - `dimension: int`
  - `SupportsBatchMotionCheck[S]`
    - `is_motion_valid_batch(edges: list[tuple[S, S]]) -> list[bool]`
- Goal abstraction
  - `GoalRegion[S]`
    - `contains(x: S) -> bool`
  - `DistanceAwareGoalRegion[S]`
    - `distance_to_goal(x: S) -> Float`
  - `GoalState[S]` (concrete region wrapper)
- Optional objective
  - `Objective[S]`
    - `path_cost(path: Sequence[S], space: ContinuousSpace[S]) -> Float`
- Problem wrapper
  - `ContinuousProblem[S]`
    - `space: ContinuousSpace[S]`
    - `start: S`
    - `goal: GoalRegion[S]`
    - `objective: Objective[S] | None`
    - `params: Mapping[str, object] | None`

## Planner-to-contract mapping

### Public entrypoints

- `pathplanning.api.plan_discrete(...)`
  - accepts `DiscreteProblem[...]`
  - dispatches to registry discrete planners, including `astar`, `dijkstra`, and
    `bidirectional_dijkstra`, and `bidirectional_astar`
- `pathplanning.api.plan_continuous(...)`
  - accepts `ContinuousProblem[...]`
  - dispatches through thin Python wrappers to native C RRT, RRT*, Informed RRT*, FMT*, BIT*, ABIT*, and RRT-Connect kernels

### Search planners

- `pathplanning.planners.search.astar.plan_astar`
  - `DiscreteProblem[N]` + `DiscreteGraph[N]`
  - optional `HeuristicDiscreteGraph[N]`
- `pathplanning.planners.search.dijkstra.plan_dijkstra`
  - `DiscreteProblem[N]` + `DiscreteGraph[N]`
- `pathplanning.planners.search.bidirectional_dijkstra.plan_bidirectional_dijkstra`
  - exact-goal search over forward and reverse native CSR adjacency
- `pathplanning.planners.search.bidirectional_astar.plan_bidirectional_astar`
  - exact-goal search using a consistent heuristic and potential-reweighted CSR edges

### Sampling planners

- `pathplanning.planners.sampling.rrt.RrtPlanner`
  - `ContinuousSpace[State]`, `GoalRegion[State]`
  - built-in 2D and 3D spaces are serialized once and checked directly in C; custom spaces use the compatibility callbacks
- `pathplanning.planners.sampling.rrt_star.RrtStarPlanner`
  - same contracts as `RrtPlanner`
- `pathplanning.planners.sampling.dynamic_rrt.DynamicRRT3D`
  - Python state/result adapter around native C tree pruning, nearest-neighbor search, and growth
- `pathplanning.planners.sampling.informed_rrt_star.InformedRrtStar`
  - RRT* with direct prolate-hyperspheroid sampling after the first solution
- `pathplanning.planners.sampling.fmt_star.FmtStar`
  - fixed-sample lazy dynamic-programming tree construction
- `pathplanning.planners.sampling.bit_star.BITStar`
  - batched samples and ordered edge search in an implicit random geometric graph
- `pathplanning.planners.sampling.abit_star.ABITStar`
  - BIT* search with decreasing heuristic inflation and truncation across batches
- `pathplanning.planners.sampling.rrt_connect.RrtConnect`
  - alternating start/goal trees with greedy connection attempts

FMT*, BIT*, ABIT*, and Informed RRT* require exact point goals, Euclidean
state-space distance, and additive path length. They reject custom objectives
because those objectives do not provide the additive lower bounds required by
their search. BIT* and ABIT* use `sample_count` and `batch_size`; FMT* uses
`sample_count` for its fixed set.

The native C++ graph-search engine owns CSR storage, search state, and frontier
queues. The native C sampling engine owns planner expansion loops, tree storage,
edge queues, and its incremental nearest-neighbor forest. Python adapts API
values and results. User-defined spaces, goals, and supported RRT* objectives
use callbacks; the built-in 2D and 3D spaces are passed as native bounds and
obstacle arrays.

### Search algorithm labels

- `bidirectional_dijkstra` expands two uniform-cost frontiers over forward and reverse CSR adjacency.
- `bidirectional_astar` applies a consistent heuristic as a potential, then runs native bidirectional Dijkstra on non-negative reduced costs. It checks consistency over the prepared graph before search.
- Both bidirectional search planners require an exact goal node.

## Reference environment implementations

Reference spaces live outside planner modules:

- `pathplanning/spaces/grid2d.py`
  - `Grid2DSearchSpace`: `DiscreteGraph` reference implementation
  - `Grid2DSamplingSpace`: `ContinuousSpace`-style 2D sampling reference
- `pathplanning/spaces/continuous_3d.py`
  - `ContinuousSpace3D`: `ContinuousSpace` reference implementation

Examples for user-defined environments:

- `examples/worlds/custom_grid_world.py`
- `examples/worlds/demo_3d_world.py`

## Non-goals

- No forced grid-only representation.
- No hardcoded plotting/visualization in planner contracts.
- No fixed obstacle primitive format.
- No heavy runtime dependencies (ROS/simulator stacks) in core planner modules.
