# Supported Planner Matrix

This matrix defines the production planner surface for `pathplanning`.

Each planner registered in `pathplanning.registry.PLANNER_REGISTRY` is
supported. The matrix and planner-specific constraints below are generated
from that registry.

<!-- BEGIN GENERATED PLANNER MATRIX -->
| Problem Kind | Planner | Module | Callable | Status |
| ------------ | ------- | ------ | -------- | ------ |
| `temporal` | `sipp` | `pathplanning.planners.temporal.sipp` | `plan_sipp` | supported |
| `temporal` | `bounded_suboptimal_sipp` | `pathplanning.planners.temporal.bounded_suboptimal_sipp` | `plan_bounded_suboptimal_sipp` | supported |
| `temporal` | `kinodynamic_sipp` | `pathplanning.planners.temporal.kinodynamic_sipp` | `plan_kinodynamic_sipp` | supported |
| `multi_agent` | `eecbs` | `pathplanning.planners.multi_agent.eecbs` | `plan_eecbs` | supported |
| `multi_agent` | `lacam_star` | `pathplanning.planners.multi_agent.lacam_star` | `plan_lacam_star` | supported |
| `discrete` | `bfs` | `pathplanning.planners.search.breadth_first_search` | `plan_breadth_first_search` | supported |
| `discrete` | `dfs` | `pathplanning.planners.search.depth_first_search` | `plan_depth_first_search` | supported |
| `discrete` | `greedy_best_first` | `pathplanning.planners.search.greedy_best_first` | `plan_greedy_best_first` | supported |
| `discrete` | `astar` | `pathplanning.planners.search.astar` | `plan_astar` | supported |
| `discrete` | `bidirectional_dijkstra` | `pathplanning.planners.search.bidirectional_dijkstra` | `plan_bidirectional_dijkstra` | supported |
| `discrete` | `bidirectional_astar` | `pathplanning.planners.search.bidirectional_astar` | `plan_bidirectional_astar` | supported |
| `discrete` | `dijkstra` | `pathplanning.planners.search.dijkstra` | `plan_dijkstra` | supported |
| `discrete` | `dstar_lite` | `pathplanning.planners.search.dstar_lite` | `plan_dstar_lite` | supported |
| `discrete` | `weighted_astar` | `pathplanning.planners.search.weighted_astar` | `plan_weighted_astar` | supported |
| `discrete` | `reexp_astar` | `pathplanning.planners.search.reexp_astar` | `plan_reexp_astar` | supported |
| `discrete` | `anytime_astar` | `pathplanning.planners.search.anytime_astar` | `plan_anytime_astar` | supported |
| `discrete` | `jps` | `pathplanning.planners.search.jump_point` | `plan_jps` | supported |
| `discrete` | `jpsw` | `pathplanning.planners.search.jump_point_weighted` | `plan_jpsw` | supported |
| `discrete` | `theta_star` | `pathplanning.planners.search.theta_star` | `plan_theta_star` | supported |
| `discrete` | `lazy_theta_star` | `pathplanning.planners.search.lazy_theta_star` | `plan_lazy_theta_star` | supported |
| `continuous` | `rrt` | `pathplanning.planners.sampling.rrt` | `plan_rrt` | supported |
| `continuous` | `rrt_star` | `pathplanning.planners.sampling.rrt_star` | `plan_rrt_star` | supported |
| `continuous` | `informed_rrt_star` | `pathplanning.planners.sampling.informed_rrt_star` | `plan_informed_rrt_star` | supported |
| `continuous` | `fmt_star` | `pathplanning.planners.sampling.fmt_star` | `plan_fmt_star` | supported |
| `continuous` | `prm_star` | `pathplanning.planners.sampling.prm_star` | `plan_prm_star` | supported |
| `continuous` | `lazy_prm` | `pathplanning.planners.sampling.prm_star` | `plan_lazy_prm` | supported |
| `continuous` | `ait_star` | `pathplanning.planners.sampling.ait_star` | `plan_ait_star` | supported |
| `continuous` | `eit_star` | `pathplanning.planners.sampling.eit_star` | `plan_eit_star` | supported |
| `continuous` | `bit_star` | `pathplanning.planners.sampling.bit_star` | `plan_bit_star` | supported |
| `continuous` | `abit_star` | `pathplanning.planners.sampling.abit_star` | `plan_abit_star` | supported |
| `continuous` | `rrt_connect` | `pathplanning.planners.sampling.rrt_connect` | `plan_rrt_connect` | supported |

Planner-specific constraints:

- `sipp`: requires positive finite edge durations and half-open node/edge blocked intervals; waiting is allowed and the goal must be exact.
- `bounded_suboptimal_sipp`: requires the same exact-goal temporal graph contract as SIPP; `w` must be finite and at least 1; the w bound applies when stop_reason is success.
- `kinodynamic_sipp`: requires integer time ticks, velocity-labeled configurations, constant-acceleration motion primitives, and finite speed/acceleration/deceleration limits; waiting is allowed only at zero-speed nodes.
- `eecbs`: requires an undirected unit-weight graph, unique starts/goals, vertex and edge-swap conflict rules, and a finite suboptimality weight `w >= 1`.
- `lacam_star`: requires an undirected unit-weight graph, unique starts/goals, vertex and edge-swap conflict rules, and stays at each reached goal; the seeded anytime search returns an optimal solution when it exhausts the open set.
- `bidirectional_dijkstra`: uses two Dijkstra frontiers and does not evaluate a heuristic; it requires an exact goal node.
- `bidirectional_astar`: runs the same native kernel over potential-reweighted edges and requires a consistent heuristic and an exact goal node.
- `dstar_lite`: requires an exact goal and non-negative costs on stable directed edges; graph heuristics must be admissible and consistent.
- `reexp_astar`: Weighted A* with conditional closed-node re-expansion; r uses r_mode (abs, rel_edge, or rel_g), and tie_break is g_low or g_high.
- `jps`: requires uniform Grid2DSearchSpace costs, standard 8-connected motions, and prohibits diagonal corner cutting.
- `jpsw`: requires TerrainCostGrid2D with positive finite terrain costs, standard 8-connected motions, exact cell goals, and no diagonal corner cutting.
- `theta_star`: requires standard 8-connected Grid2DSearchSpace or TerrainCostGrid2D; line of sight rejects blocked cells and corner cutting.
- `lazy_theta_star`: requires standard 8-connected Grid2DSearchSpace or TerrainCostGrid2D; candidate shortcuts are validated when nodes are expanded.
- `informed_rrt_star`, `fmt_star`, `bit_star`, `abit_star`: require exact point goals, Euclidean state-space distance, and additive path length; custom objectives are rejected because the search bounds rely on additive path length.
- `fmt_star`: uses `sample_count` as its fixed sample set size.
- `prm_star`: requires an exact point goal, Euclidean metric, additive path length, symmetric local motion validity, and a positive roadmap gamma; custom Python callbacks are opt-in through RoadmapParams.
- `lazy_prm`: requires an exact point goal, Euclidean metric, additive path length, symmetric local motion validity, and a positive roadmap gamma; edges are validated lazily and cached, while custom Python callbacks are opt-in through RoadmapParams.
- `ait_star`: requires an exact point goal, Euclidean metric, additive path length, and a positive sample count; collision discovery repairs its reverse graph heuristic.
- `eit_star`: requires an exact point goal, Euclidean metric, and additive path length; collision-check effort from collision_step breaks ties between cost estimates.
- `bit_star`, `abit_star`: use `sample_count` across batches and `batch_size` per batch.
<!-- END GENERATED PLANNER MATRIX -->

## Native Execution

All registry-backed discrete searches use the C++ graph-search engine. All
registry-backed continuous planners use the C sampling engine; Python planner
modules validate inputs and adapt the public API to the native C ABI. Built-in
spaces and custom spaces that return `NativeContinuousSpaceModel` run geometry,
collision checks, and search without Python callbacks. Callback compatibility
for custom spaces, goal predicates, and RRT* objectives requires the explicit
`RrtParams(allow_python_callbacks=True)` opt-in.

`DynamicRRT3D` is a stateful Python API outside the registry matrix. Its tree
pruning, edge checks, nearest-node search, and growth run in the same native C
sampling engine. See [`docs/native_core.md`](docs/native_core.md) for data
ownership, build steps, callback behavior, and benchmark limitations.

Regenerate the planner matrix after registry changes with
`python scripts/generate_supported_algorithms.py`. CI checks that this file is
up to date.
