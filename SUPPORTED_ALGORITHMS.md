# Supported Algorithms Matrix

This matrix defines the production algorithm surface for `PathPlanning`.

Status values:

- `supported`: included in production package API/smoke tests
- `dropped_incomplete`: excluded from production surface until completed

## Search 2D

| Algorithm ID                       | Module                                                              | Status    |
| ---------------------------------- | ------------------------------------------------------------------- | --------- |
| `search2d.breadth_first_search`    | `pathplanning.Search_based_Planning.plan2d.breadth_first_search`    | supported |
| `search2d.depth_first_search`      | `pathplanning.Search_based_Planning.plan2d.depth_first_search`      | supported |
| `search2d.best_first_search`       | `pathplanning.Search_based_Planning.plan2d.best_first_search`       | supported |
| `search2d.dijkstra`                | `pathplanning.Search_based_Planning.plan2d.dijkstra`                | supported |
| `search2d.astar`                   | `pathplanning.Search_based_Planning.plan2d.astar`                   | supported |
| `search2d.bidirectional_astar`     | `pathplanning.Search_based_Planning.plan2d.bidirectional_astar`     | supported |
| `search2d.anytime_dstar`           | `pathplanning.Search_based_Planning.plan2d.anytime_dstar`           | supported |
| `search2d.anytime_repairing_astar` | `pathplanning.Search_based_Planning.plan2d.anytime_repairing_astar` | supported |
| `search2d.lifelong_planning_astar` | `pathplanning.Search_based_Planning.plan2d.lifelong_planning_astar` | supported |
| `search2d.learning_realtime_astar` | `pathplanning.Search_based_Planning.plan2d.learning_realtime_astar` | supported |
| `search2d.realtime_adaptive_astar` | `pathplanning.Search_based_Planning.plan2d.realtime_adaptive_astar` | supported |
| `search2d.dstar_lite`              | `pathplanning.Search_based_Planning.plan2d.dstar_lite`              | supported |
| `search2d.dstar`                   | `pathplanning.Search_based_Planning.plan2d.dstar`                   | supported |

## Search 3D

| Algorithm ID                       | Module                                                               | Status    |
| ---------------------------------- | -------------------------------------------------------------------- | --------- |
| `search3d.astar`                   | `pathplanning.Search_based_Planning.Search_3D.Astar3D`               | supported |
| `search3d.bidirectional_astar`     | `pathplanning.Search_based_Planning.Search_3D.bidirectional_Astar3D` | supported |
| `search3d.lifelong_planning_astar` | `pathplanning.Search_based_Planning.Search_3D.LP_Astar3D`            | supported |
| `search3d.learning_realtime_astar` | `pathplanning.Search_based_Planning.Search_3D.LRT_Astar3D`           | supported |
| `search3d.realtime_adaptive_astar` | `pathplanning.Search_based_Planning.Search_3D.RTA_Astar3D`           | supported |
| `search3d.dstar`                   | `pathplanning.Search_based_Planning.Search_3D.Dstar3D`               | supported |
| `search3d.dstar_lite`              | `pathplanning.Search_based_Planning.Search_3D.DstarLite3D`           | supported |
| `search3d.anytime_dstar`           | `pathplanning.Search_based_Planning.Search_3D.Anytime_Dstar3D`       | supported |

## Sampling 2D

| Algorithm ID                               | Module                                                                      | Status             |
| ------------------------------------------ | --------------------------------------------------------------------------- | ------------------ |
| `sampling2d.rrt`                           | `pathplanning.Sampling_based_Planning.rrt_2D.rrt`                           | supported          |
| `sampling2d.rrt_connect`                   | `pathplanning.Sampling_based_Planning.rrt_2D.rrt_connect`                   | supported          |
| `sampling2d.extended_rrt`                  | `pathplanning.Sampling_based_Planning.rrt_2D.extended_rrt`                  | supported          |
| `sampling2d.dynamic_rrt`                   | `pathplanning.Sampling_based_Planning.rrt_2D.dynamic_rrt`                   | supported          |
| `sampling2d.rrt_star`                      | `pathplanning.Sampling_based_Planning.rrt_2D.rrt_star`                      | supported          |
| `sampling2d.rrt_sharp`                     | `pathplanning.Sampling_based_Planning.rrt_2D.rrt_sharp`                     | dropped_incomplete |
| `sampling2d.informed_rrt_star`             | `pathplanning.Sampling_based_Planning.rrt_2D.informed_rrt_star`             | supported          |
| `sampling2d.rrt_star_smart`                | `pathplanning.Sampling_based_Planning.rrt_2D.rrt_star_smart`                | supported          |
| `sampling2d.dubins_rrt_star`               | `pathplanning.Sampling_based_Planning.rrt_2D.dubins_rrt_star`               | supported          |
| `sampling2d.fast_marching_trees`           | `pathplanning.Sampling_based_Planning.rrt_2D.fast_marching_trees`           | supported          |
| `sampling2d.batch_informed_trees`          | `pathplanning.Sampling_based_Planning.rrt_2D.batch_informed_trees`          | supported          |
| `sampling2d.advanced_batch_informed_trees` | `pathplanning.Sampling_based_Planning.rrt_2D.advanced_batch_informed_trees` | dropped_incomplete |
| `sampling2d.adaptively_informed_trees`     | `pathplanning.Sampling_based_Planning.rrt_2D.adaptively_informed_trees`     | dropped_incomplete |

## Sampling 3D

| Algorithm ID                   | Module                                                            | Status             |
| ------------------------------ | ----------------------------------------------------------------- | ------------------ |
| `sampling3d.rrt`               | `pathplanning.Sampling_based_Planning.rrt_3D.rrt3D`               | supported          |
| `sampling3d.rrt_connect`       | `pathplanning.Sampling_based_Planning.rrt_3D.rrt_connect3D`       | supported          |
| `sampling3d.extended_rrt`      | `pathplanning.Sampling_based_Planning.rrt_3D.extend_rrt3D`        | supported          |
| `sampling3d.dynamic_rrt`       | `pathplanning.Sampling_based_Planning.rrt_3D.dynamic_rrt3D`       | supported          |
| `sampling3d.rrt_star`          | `pathplanning.Sampling_based_Planning.rrt_3D.rrt_star3D`          | supported          |
| `sampling3d.informed_rrt_star` | `pathplanning.Sampling_based_Planning.rrt_3D.informed_rrt_star3D` | supported          |
| `sampling3d.rrt_star_smart`    | `pathplanning.Sampling_based_Planning.rrt_3D.rrt_star_smart3D`    | dropped_incomplete |
| `sampling3d.fmt_star`          | `pathplanning.Sampling_based_Planning.rrt_3D.FMT_star3D`          | supported          |
| `sampling3d.bit_star`          | `pathplanning.Sampling_based_Planning.rrt_3D.BIT_star3D`          | supported          |
| `sampling3d.abit_star`         | `pathplanning.Sampling_based_Planning.rrt_3D.ABIT_star3D`         | dropped_incomplete |

Dropped reason for `sampling3d.abit_star`:

- The implementation has undefined symbols and stubbed methods.
