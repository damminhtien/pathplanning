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
| `discrete`   | `bidirectional_astar` | `pathplanning.planners.search.bidirectional_astar` | `plan_bidirectional_astar` | supported |
| `discrete`   | `dijkstra`          | `pathplanning.planners.search.dijkstra`     | `plan_dijkstra`         | supported |
| `discrete`   | `weighted_astar`    | `pathplanning.planners.search.weighted_astar` | `plan_weighted_astar` | supported |
| `discrete`   | `anytime_astar`     | `pathplanning.planners.search.anytime_astar` | `plan_anytime_astar` | supported |
| `continuous` | `rrt`               | `pathplanning.planners.sampling.rrt`        | `plan_rrt`              | supported |
| `continuous` | `rrt_star`          | `pathplanning.planners.sampling.rrt_star`   | `plan_rrt_star`         | supported |
| `continuous` | `informed_rrt_star` | `pathplanning.planners.sampling.informed_rrt_star` | `plan_informed_rrt_star` | supported |
| `continuous` | `bit_star`          | `pathplanning.planners.sampling.bit_star`   | `plan_bit_star`         | supported |
| `continuous` | `fmt_star`          | `pathplanning.planners.sampling.fmt_star`   | `plan_fmt_star`         | supported |
| `continuous` | `rrt_connect`       | `pathplanning.planners.sampling.rrt_connect` | `plan_rrt_connect` | supported |
| `continuous` | `abit_star`         | `pathplanning.planners.sampling.abit_star`  | `plan_abit_star`        | supported |
