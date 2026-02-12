"""Time budget behavior tests for RRT planners."""

from __future__ import annotations

import numpy as np

from pathplanning.core.params import RrtParams
from pathplanning.core.results import StopReason
from pathplanning.spaces.continuous_3d import AABB, ContinuousSpace3D
from pathplanning.sampling_based_planning.rrt_3d.rrt import RrtPlanner
from pathplanning.sampling_based_planning.rrt_3d.rrt_star import RrtStarPlanner


def _build_space() -> ContinuousSpace3D:
    return ContinuousSpace3D(
        lower_bound=[0.0, 0.0, 0.0],
        upper_bound=[10.0, 10.0, 10.0],
        aabbs=[AABB([4.0, 4.0, 0.0], [6.0, 6.0, 8.0])],
    )


def test_rrt_time_budget_stops_early() -> None:
    start = np.array([1.0, 1.0, 1.0], dtype=float)
    goal_center = np.array([9.0, 9.0, 1.0], dtype=float)
    params = RrtParams(
        max_iters=10_000,
        step_size=0.6,
        goal_sample_rate=0.05,
        time_budget_s=1e-6,
        max_sample_tries=2_000,
        collision_step=0.1,
    )

    result = RrtPlanner(_build_space(), params, np.random.default_rng(7)).plan(
        start, (goal_center, 0.25)
    )

    assert result.stop_reason is StopReason.TIME_BUDGET
    assert result.iters < params.max_iters
    assert result.nodes >= 1


def test_rrt_star_time_budget_stops_early() -> None:
    start = np.array([1.0, 1.0, 1.0], dtype=float)
    goal_center = np.array([9.0, 9.0, 1.0], dtype=float)
    params = RrtParams(
        max_iters=10_000,
        step_size=0.6,
        goal_sample_rate=0.05,
        time_budget_s=1e-6,
        max_sample_tries=2_000,
        collision_step=0.1,
    )

    result = RrtStarPlanner(_build_space(), params, np.random.default_rng(7)).plan(
        start, (goal_center, 0.25)
    )

    assert result.stop_reason is StopReason.TIME_BUDGET
    assert result.iters < params.max_iters
    assert result.nodes >= 1
