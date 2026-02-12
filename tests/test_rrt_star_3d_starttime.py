from __future__ import annotations

import numpy as np

from pathplanning.core.contracts import GoalState
from pathplanning.core.params import RrtParams
from pathplanning.planners.sampling.rrt_star import RrtStarPlanner
from pathplanning.spaces.continuous_3d import ContinuousSpace3D


def test_rrt_star_3d_plan_has_no_global_starttime_dependency() -> None:
    """RRT* planning should run without relying on module-level timer globals."""
    space = ContinuousSpace3D(
        lower_bound=[0.0, 0.0, 0.0],
        upper_bound=[1.0, 1.0, 1.0],
    )
    params = RrtParams(
        max_iters=10,
        step_size=0.2,
        goal_sample_rate=0.2,
        max_sample_tries=100,
        collision_step=0.05,
    )

    start = np.array([0.1, 0.1, 0.1], dtype=float)
    goal = np.array([0.9, 0.9, 0.9], dtype=float)
    goal_region = GoalState(state=goal, radius=0.1, distance_fn=space.distance)
    result = RrtStarPlanner(space, params, np.random.default_rng(7)).plan(start, goal_region)

    assert result.iters >= 0
