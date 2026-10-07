# Supported Planner Matrix

This matrix defines the production planner surface for `pathplanning`.

Each planner registered in `pathplanning.registry.PLANNER_REGISTRY` is
supported. The matrix and planner-specific constraints below are generated
from that registry.

<!-- BEGIN GENERATED PLANNER MATRIX -->
| Problem Kind | Planner | Module | Callable | Status |
| ------------ | ------- | ------ | -------- | ------ |
| `discrete` | `bfs` | `pathplanning.planners.search.breadth_first_search` | `plan_breadth_first_search` | supported |
| `discrete` | `dfs` | `pathplanning.planners.search.depth_first_search` | `plan_depth_first_search` | supported |
| `discrete` | `greedy_best_first` | `pathplanning.planners.search.greedy_best_first` | `plan_greedy_best_first` | supported |
| `discrete` | `astar` | `pathplanning.planners.search.astar` | `plan_astar` | supported |
| `discrete` | `bidirectional_dijkstra` | `pathplanning.planners.search.bidirectional_dijkstra` | `plan_bidirectional_dijkstra` | supported |
| `discrete` | `bidirectional_astar` | `pathplanning.planners.search.bidirectional_astar` | `plan_bidirectional_astar` | supported |
| `discrete` | `dijkstra` | `pathplanning.planners.search.dijkstra` | `plan_dijkstra` | supported |
| `discrete` | `weighted_astar` | `pathplanning.planners.search.weighted_astar` | `plan_weighted_astar` | supported |
| `discrete` | `reexp_astar` | `pathplanning.planners.search.reexp_astar` | `plan_reexp_astar` | supported |
| `discrete` | `anytime_astar` | `pathplanning.planners.search.anytime_astar` | `plan_anytime_astar` | supported |
| `discrete` | `jps` | `pathplanning.planners.search.jump_point` | `plan_jps` | supported |
| `discrete` | `jpsw` | `pathplanning.planners.search.jump_point_weighted` | `plan_jpsw` | supported |
| `continuous` | `rrt` | `pathplanning.planners.sampling.rrt` | `plan_rrt` | supported |
| `continuous` | `rrt_star` | `pathplanning.planners.sampling.rrt_star` | `plan_rrt_star` | supported |
| `continuous` | `informed_rrt_star` | `pathplanning.planners.sampling.informed_rrt_star` | `plan_informed_rrt_star` | supported |
| `continuous` | `fmt_star` | `pathplanning.planners.sampling.fmt_star` | `plan_fmt_star` | supported |
| `continuous` | `bit_star` | `pathplanning.planners.sampling.bit_star` | `plan_bit_star` | supported |
| `continuous` | `abit_star` | `pathplanning.planners.sampling.abit_star` | `plan_abit_star` | supported |
| `continuous` | `rrt_connect` | `pathplanning.planners.sampling.rrt_connect` | `plan_rrt_connect` | supported |

Planner-specific constraints:

- `bidirectional_dijkstra`: uses two Dijkstra frontiers and does not evaluate a heuristic; it requires an exact goal node.
- `bidirectional_astar`: runs the same native kernel over potential-reweighted edges and requires a consistent heuristic and an exact goal node.
- `reexp_astar`: Weighted A* with conditional closed-node re-expansion; r uses r_mode (abs, rel_edge, or rel_g), and tie_break is g_low or g_high.
- `jps`: requires uniform Grid2DSearchSpace costs, standard 8-connected motions, and prohibits diagonal corner cutting.
- `jpsw`: requires TerrainCostGrid2D with positive finite terrain costs, standard 8-connected motions, exact cell goals, and no diagonal corner cutting.
- `informed_rrt_star`, `fmt_star`, `bit_star`, `abit_star`: require exact point goals, Euclidean state-space distance, and additive path length; custom objectives are rejected because the search bounds rely on additive path length.
- `fmt_star`: uses `sample_count` as its fixed sample set size.
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
