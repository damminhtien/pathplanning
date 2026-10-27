"""Contract tests for the headless 3D ``RrtStarPlanner``."""

from __future__ import annotations

import numpy as np
import pytest

from pathplanning.core.contracts import GoalState
from pathplanning.core.params import RrtParams
from pathplanning.planners.sampling.rrt_star import RrtStarPlanner
from pathplanning.spaces.continuous_3d import AABB, ContinuousSpace3D

pytestmark = pytest.mark.slow


def _build_space() -> ContinuousSpace3D:
    return ContinuousSpace3D(
        lower_bound=[0.0, 0.0, 0.0],
        upper_bound=[10.0, 10.0, 10.0],
        aabbs=[AABB([4.0, 4.0, 0.0], [6.0, 6.0, 8.0])],
    )


def test_rrtstar_planner_is_deterministic_for_fixed_seed() -> None:
    start = np.array([1.0, 1.0, 1.0], dtype=float)
    goal_center = np.array([9.0, 9.0, 1.0], dtype=float)
    goal_tolerance = 0.75
    params = RrtParams(
        max_iters=3_000,
        step_size=0.6,
        goal_sample_rate=0.2,
        max_sample_tries=2_000,
        collision_step=0.1,
    )

    first_space = _build_space()
    second_space = _build_space()
    first_goal = GoalState(
        state=goal_center,
        radius=goal_tolerance,
        distance_fn=first_space.distance,
    )
    second_goal = GoalState(
        state=goal_center,
        radius=goal_tolerance,
        distance_fn=second_space.distance,
    )
    first = RrtStarPlanner(first_space, params, np.random.default_rng(7)).plan(start, first_goal)
    second = RrtStarPlanner(second_space, params, np.random.default_rng(7)).plan(start, second_goal)

    assert first.success and second.success
    assert first.path is not None
    assert second.path is not None
    assert first.nodes == second.nodes
    assert first.iters == second.iters
    assert len(first.path) == len(second.path)
    assert np.allclose(first.path[-1], second.path[-1])
    assert first.stats["path_cost"] == second.stats["path_cost"]
    assert np.allclose(first.path[0], start)
    assert first_space.distance(first.path[-1], goal_center) <= goal_tolerance
    assert all(first_space.is_motion_valid(a, b) for a, b in zip(first.path, first.path[1:]))
    cumulative_costs = np.concatenate(
        ([0.0], np.cumsum(np.linalg.norm(np.diff(first.path, axis=0), axis=1)))
    )
    assert np.isclose(cumulative_costs[-1], first.stats["path_cost"])
    assert all(
        current > previous for previous, current in zip(cumulative_costs, cumulative_costs[1:])
    )
