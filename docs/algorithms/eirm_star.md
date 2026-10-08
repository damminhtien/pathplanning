# EIRM*

EIRM* (Effort Informed Roadmaps) is a multi-query planner designed to reuse
roadmap samples and validation results. `EirmStarRoadmap` keeps one sampled
random geometric graph for a caller-managed `world_version`; it caches valid
and blocked roadmap edges across exact start/goal queries. The search uses
estimated collision-check effort as a secondary key among equal path-cost
routes, so a known-valid route can be preferred over an equally costly route
that needs more validation.

```python
import numpy as np

from pathplanning import RoadmapParams
from pathplanning.planners.sampling.eirm_star import EirmStarRoadmap
from pathplanning.spaces.grid2d import Grid2DSamplingSpace

space = Grid2DSamplingSpace(x_range=(0.0, 10.0), y_range=(0.0, 10.0))
with EirmStarRoadmap(
    space,
    2,
    RoadmapParams(sample_count=96, gamma=8.0),
    np.random.default_rng(7),
) as roadmap:
    for start, goal in (
        ((0.5, 0.5), (9.5, 9.5)),
        ((0.5, 9.5), (9.5, 0.5)),
    ):
        result = roadmap.query(start, goal, world_version="world-v1")
        print(result.success, result.stats.get("motion_checks"))
```

The initial implementation uses a fixed finite roadmap and a lexicographic
cost/effort search with lazy edge validation. It reuses edge outcomes between
queries but does not implement the paper's full asymmetric bidirectional
search or its asymptotic-optimality guarantee. The supported objective is
additive Euclidean path length, with exact point goals and symmetric motion
validity. Change `world_version` whenever the obstacle topology or metric
changes; doing so rebuilds the roadmap and clears its validation cache.

Reference: [Hartmann et al., Effort Informed Roadmaps (EIRM*): Efficient
Asymptotically Optimal Multiquery Planning by Actively Reusing Validation
Effort (ISRR 2022)](https://arxiv.org/abs/2205.08480).
