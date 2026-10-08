# Lazy PRM

Lazy PRM builds the same Euclidean random geometric graph as PRM* but treats
unvalidated roadmap edges as provisionally free. It runs a shortest-path query,
validates only edges on the candidate route, removes an edge when motion checking
fails, and searches again. Successful and failed edge checks stay on the reusable
roadmap until the caller changes `world_version` or resets it.

`LazyPrmRoadmap` exposes `build()`, `query()`, `clear_query()`, and `reset()`.
It shares `RoadmapParams` and supports exact point goals, additive Euclidean path
length, and symmetric local motion validity. `gamma` controls the connection
radius `gamma * (log(n) / n)^(1/d)`; choose it for the free-space measure and
dimension when asymptotic guarantees are required. The focused test uses a small
wall map to verify returned motions and validation-cache reuse.

```python
import numpy as np

from pathplanning import RoadmapParams
from pathplanning.planners.sampling.prm_star import LazyPrmRoadmap
from pathplanning.spaces.grid2d import Grid2DSamplingSpace

space = Grid2DSamplingSpace(x_range=(0.0, 5.0), y_range=(0.0, 5.0))
with LazyPrmRoadmap(
    space, 2, RoadmapParams(sample_count=256, gamma=8.0), np.random.default_rng(7)
) as roadmap:
    result = roadmap.query((0.5, 0.5), (4.5, 4.5), world_version="world-v1")
    print(result.success, result.path)
```

Reference: [Bohlin and Kavraki, Path Planning Using Lazy PRM (ICRA 2000)](https://www.kavrakilab.org/publications/bohlin-kavraki2000path-planning-using.pdf).
