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
  - dispatches to registry discrete planners (`astar`, `dijkstra`)
- `pathplanning.api.plan_continuous(...)`
  - accepts `ContinuousProblem[...]`
  - dispatches to registry continuous planners (`rrt`, `rrt_star`, etc.)

### Search planners

- `pathplanning.planners.search.entrypoints.plan_astar`
  - `DiscreteProblem[N]` + `DiscreteGraph[N]`
  - optional `HeuristicDiscreteGraph[N]`
- `pathplanning.planners.search.entrypoints.plan_dijkstra`
  - `DiscreteProblem[N]` + `DiscreteGraph[N]`
- `pathplanning.search2d.Search2D`
  - contract-first 2D facade over `DiscreteGraph[tuple[int, int]]`

### Sampling planners

- `pathplanning.planners.sampling.rrt.RrtPlanner`
  - `ContinuousSpace[State]`, `GoalRegion[State]`
  - optional `SupportsBatchMotionCheck[State]`
- `pathplanning.planners.sampling.rrt_star.RrtStarPlanner`
  - same contracts as `RrtPlanner`
- `pathplanning.planners.sampling.dynamic_rrt.DynamicRRT3D`
  - contract-driven dynamic wrapper around `RrtPlanner`
- `pathplanning.planners.sampling.rrt_grid2d.Rrt`
  - compatibility class backed by `RrtPlanner` + `ContinuousSpace`
- `pathplanning.planners.sampling.bit_star.BitStar`, `...informed_rrt_star.InformedRrtStar`, `...fmt_star.FmtStar`, `...abit_star.AbitStar`, `...rrt_connect.RrtConnect`
  - import-safe wrappers delegating to contract-driven core planners

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
