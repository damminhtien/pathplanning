# 3D RRT Refactor Notes

## Scope

This document describes the production refactor applied to the 3D sampling stack
(`pathplanning/sampling_based_planning/rrt_3d`).

Goals:
- keep planners import-safe in headless environments
- provide deterministic execution for tests and reproducibility
- reduce environment hard-coupling via explicit contracts
- make nearest-neighbor selection pluggable for scalability
- preserve runtime behavior unless fixing a correctness bug

## Architecture Changes

### DynamicRRT3D core contract

`DynamicRRT3D` now supports explicit runtime contracts:
- `DynamicRRT3DConfig`: planner constants and thresholds
- `environment` injection in constructor
- `rng` injection (or `DynamicRRT3D.with_seed(...)`) for deterministic sampling
- pluggable nearest index (`NearestNodeIndex` protocol)

Included nearest backends:
- `BruteForceNearestNodeIndex`
- `KDTreeNearestNodeIndex` (batched `scipy.spatial.cKDTree` rebuild)

### Headless import safety

`dynamic_rrt_3d` no longer imports plotting utilities at module import time.
Plot utilities are imported lazily inside visualization paths.

`utils_3d` now exposes a lazy wrapper for `visualization(...)` to avoid importing
`plot_util_3d` during core module import.

### Legacy wrapper removal

Legacy compatibility wrappers (`rrt` / `rrtstar`) were removed.
Registry-supported RRT entrypoints now map directly to:
- `RrtPlanner` in `pathplanning.sampling_based_planning.rrt_3d.rrt`
- `RrtStarPlanner` in `pathplanning.sampling_based_planning.rrt_3d.rrt_star`

### Environment3D canonical naming

`Environment3D` now stores canonical snake_case obstacle fields:
- `aabb`
- `aabb_pyrr`
- `obb`

Legacy names (`AABB`, `AABB_pyrr`, `OBB`) are still available as compatibility
properties.

## Correctness Fixes

- Fixed mutable default argument in `utils_3d.path(...)`.
- Fixed recursive sampling bias propagation in `utils_3d.sampleFree(...)`.
- Fixed `rrt_star_3d` timer scope bug (`starttime` now defined in `run`).

## Migration Guide

### DynamicRRT3D usage

Before:
```python
planner = DynamicRRT3D()
```

After (deterministic):
```python
planner = DynamicRRT3D.with_seed(7)
```

After (explicit contract):
```python
config = DynamicRRT3DConfig(step_size=0.25, max_iterations=10000)
planner = DynamicRRT3D(environment=my_env, config=config, rng=my_rng)
```

### Environment fields

Preferred:
```python
env.aabb
env.aabb_pyrr
env.obb
```

Compatibility (deprecated):
```python
env.AABB
env.AABB_pyrr
env.OBB
```

## Test Coverage Added

- dynamic RRT import remains headless-safe
- dynamic RRT choose-target behavior is deterministic with fixed seed
- brute-force and KDTree nearest backends agree on nearest-node results
- Environment3D canonical/deprecated contract behavior is validated
