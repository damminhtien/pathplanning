# Project State

Last updated: 2026-02-12
Branch: `main`

## Current Snapshot

- Project: `PathPlanning`
- Package: `pathplanning`
- Version: `0.2.0`
- Canonical repository: `https://github.com/damminhtien/pathplanning`
- Python requirement: `>=3.10`
- Runtime deps: `requirements.txt`
- Dev deps: `requirements-dev.txt`

## Production API Surface

Registry-backed supported algorithms:

1. `sampling3d.rrt` -> `pathplanning.planners.sampling.rrt:RrtPlanner`
2. `sampling3d.rrt_star` -> `pathplanning.planners.sampling.rrt_star:RrtStarPlanner`

## Architecture Snapshot

- Shared space/environment layer: `pathplanning/spaces`
- Shared NN index: `pathplanning/nn/index.py`
- Shared tree storage: `pathplanning/data_structures/tree_array.py`
- Shared priority queue: `pathplanning/utils/priority_queue.py`
- Geometry math: `pathplanning/geometry`
- Plotting helpers: `pathplanning/viz`

## Validation Baseline

Most recent full run observed:

- `pytest -q` -> `67 passed`

## Active Risks / Gaps

1. CI workflow still references some legacy paths and should be aligned with the new layout.
2. Production registry scope is intentionally narrow; adding algorithms requires explicit registry + test coverage updates.
3. Several root docs were previously stale and require periodic synchronization after large refactors.

## Next High-Value Tasks

1. Align `.github/workflows/pylint.yml` targets with `planners/spaces/geometry` layout.
2. Expand typed/registry-backed production surface beyond current 3D RRT pair.
3. Add benchmark baselines for key search and sampling planners.
