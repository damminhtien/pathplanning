# Supported Planner Matrix

This matrix defines the production planner surface for `pathplanning`.

Status values:

- `supported`: included in production package API and registry integrity tests

| Problem Kind | Planner             | Module                                 | Callable                 | Status    |
| ------------ | ------------------- | -------------------------------------- | ------------------------ | --------- |
| `discrete`   | `bfs`               | `pathplanning.planners.search.breadth_first_search` | `plan_breadth_first_search` | supported |
| `discrete`   | `dfs`               | `pathplanning.planners.search.depth_first_search` | `plan_depth_first_search` | supported |
| `discrete`   | `greedy_best_first` | `pathplanning.planners.search.greedy_best_first` | `plan_greedy_best_first` | supported |
| `discrete`   | `astar`             | `pathplanning.planners.search.astar`        | `plan_astar`            | supported |
| `discrete`   | `bidirectional_dijkstra` | `pathplanning.planners.search.bidirectional_dijkstra` | `plan_bidirectional_dijkstra` | supported |
| `discrete`   | `dijkstra`          | `pathplanning.planners.search.dijkstra`     | `plan_dijkstra`         | supported |
| `discrete`   | `weighted_astar`    | `pathplanning.planners.search.weighted_astar` | `plan_weighted_astar` | supported |
| `discrete`   | `anytime_astar`     | `pathplanning.planners.search.anytime_astar` | `plan_anytime_astar` | supported |
| `continuous` | `rrt`               | `pathplanning.planners.sampling.rrt`        | `plan_rrt`              | supported |
| `continuous` | `rrt_star`          | `pathplanning.planners.sampling.rrt_star`   | `plan_rrt_star`         | supported |

`bidirectional_dijkstra` uses two Dijkstra frontiers and does not evaluate a
heuristic. It requires an exact goal node. The old registered name
`bidirectional_astar` has been removed because it described a different
algorithm; the old module path remains as a clearly documented import alias.

## Compatibility entry points not registered as supported

These import paths remain available for source compatibility, but each delegates
to another algorithm. They are excluded from the registry and from the supported
planner surface:

| Legacy name | Actual implementation |
| ----------- | --------------------- |
| `informed_rrt_star` | RRT* without informed sampling |
| `bit_star` | RRT* |
| `abit_star` | RRT* |
| `fmt_star` | RRT |
| `rrt_connect` | RRT |
