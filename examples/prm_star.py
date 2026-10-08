"""Build and query a reusable PRM* roadmap."""

import numpy as np

from pathplanning import RoadmapParams
from pathplanning.planners.sampling.prm_star import PrmStarRoadmap
from pathplanning.spaces.grid2d import Grid2DSamplingSpace

space = Grid2DSamplingSpace(x_range=(0.0, 10.0), y_range=(0.0, 10.0))
with PrmStarRoadmap(
    space,
    dimension=2,
    params=RoadmapParams(sample_count=256, gamma=8.0),
    rng=np.random.default_rng(7),
) as roadmap:
    roadmap.build(world_version=1)
    for start, goal in (((1.0, 1.0), (9.0, 9.0)), ((1.0, 9.0), (9.0, 1.0))):
        result = roadmap.query(start, goal, world_version=1)
        print("success:", result.success)
        print("path cost:", result.stats.get("path_cost"))
