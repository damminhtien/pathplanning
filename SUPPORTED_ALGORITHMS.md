# Supported Algorithms Matrix

This matrix defines the production algorithm surface for `pathplanning`.

Status values:

- `supported`: included in production package API and registry integrity tests

## Sampling 3D

| Algorithm ID          | Module                                    | Entrypoint       | Status    |
| --------------------- | ----------------------------------------- | ---------------- | --------- |
| `sampling3d.rrt`      | `pathplanning.planners.sampling.rrt`      | `RrtPlanner`     | supported |
| `sampling3d.rrt_star` | `pathplanning.planners.sampling.rrt_star` | `RrtStarPlanner` | supported |
