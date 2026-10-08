# PRM*

PRM* builds a reusable random geometric roadmap in the native C core, then
connects exact query start and goal states and runs Dijkstra on the resulting
graph. It supports additive Euclidean path length and symmetric local motion
validity. The connection radius is
`gamma * (log(n) / n)^(1/d)`, using the number of retained samples and the
continuous state dimension. Choose `gamma` large enough for the free-space
measure and dimension when asymptotic optimality is required.

`PrmStarRoadmap` exposes `build()`, `query()`, `clear_query()`, and `reset()`.
Queries retain the built roadmap. A caller-managed `world_version` change
rebuilds it so obstacle, topology, or metric changes cannot reuse stale edges.
`RoadmapParams` controls sample count, gamma, collision step, query expansions,
time budget, and callback opt-in. Custom Python space callbacks run only when
`allow_python_callbacks=True`; native built-in and declared models stay in C.

```python
import numpy as np

from pathplanning import RoadmapParams
from pathplanning.core.trace import TraceOptions
from pathplanning.planners.sampling.prm_star import PrmStarRoadmap
from pathplanning.spaces.grid2d import Grid2DSamplingSpace

space = Grid2DSamplingSpace(x_range=(0.0, 10.0), y_range=(0.0, 10.0))
params = RoadmapParams(sample_count=512, gamma=8.0)
with PrmStarRoadmap(space, 2, params, np.random.default_rng(7)) as roadmap:
    roadmap.build(world_version="empty-world-v1")
    result = roadmap.query(
        (1.0, 1.0),
        (9.0, 9.0),
        world_version="empty-world-v1",
        trace=TraceOptions(32_768),
    )
    print(result.path, result.stats.get("path_cost"))
```

The planner returns a feasible incumbent when the query budget stops after a
path is found. Roadmap construction and each query report separate sample,
motion-check, expansion, and timing counters. Small tests validate multi-query
reuse and path validity; they do not prove asymptotic guarantees.

Reference: [Karaman and Frazzoli, Sampling-based Algorithms for Optimal Motion Planning (IJRR 2011)](https://arxiv.org/abs/1105.1186).
