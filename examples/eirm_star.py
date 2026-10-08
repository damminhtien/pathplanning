"""Reuse an effort-informed roadmap across multiple planning queries."""

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
        print("success:", result.success)
        print("motion checks:", result.stats.get("motion_checks"))
