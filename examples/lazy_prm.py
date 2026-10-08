"""Plan repeated queries with a reusable lazy PRM roadmap."""

import numpy as np

from pathplanning import RoadmapParams
from pathplanning.planners.sampling.prm_star import LazyPrmRoadmap
from pathplanning.spaces.grid2d import Grid2DSamplingSpace

space = Grid2DSamplingSpace(
    x_range=(0.0, 5.0),
    y_range=(0.0, 5.0),
    obs_rectangle=([2, 0, 1, 4],),
    delta=0.0,
)
with LazyPrmRoadmap(
    space,
    2,
    RoadmapParams(sample_count=128, gamma=8.0),
    np.random.default_rng(7),
) as roadmap:
    for start, goal in (((0.5, 2.5), (4.5, 2.5)), ((0.5, 1.5), (4.5, 1.5))):
        result = roadmap.query(start, goal, world_version="wall-v1")
        print("success:", result.success)
        print("path cost:", result.stats.get("path_cost"))
