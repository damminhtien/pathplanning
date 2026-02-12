"""Contract tests for the headless 3D ``RrtPlanner``."""

from __future__ import annotations

import numpy as np

from pathplanning.core.contracts import GoalState
from pathplanning.core.params import RrtParams
from pathplanning.planners.sampling.rrt import RrtPlanner
from pathplanning.spaces.continuous_3d import AABB, ContinuousSpace3D


def _build_space() -> ContinuousSpace3D:
    return ContinuousSpace3D(
        lower_bound=[0.0, 0.0, 0.0],
        upper_bound=[10.0, 10.0, 10.0],
        aabbs=[AABB([4.0, 4.0, 0.0], [6.0, 6.0, 8.0])],
    )


def test_rrt_planner_is_deterministic_for_fixed_seed() -> None:
    start = np.array([1.0, 1.0, 1.0], dtype=float)
    goal_center = np.array([9.0, 9.0, 1.0], dtype=float)
    goal_tolerance = 0.75
    params = RrtParams(
        max_iters=2_000,
        step_size=0.6,
        goal_sample_rate=0.1,
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
    first = RrtPlanner(first_space, params, np.random.default_rng(7)).plan(start, first_goal)
    second = RrtPlanner(second_space, params, np.random.default_rng(7)).plan(start, second_goal)

    assert first.success and second.success
    assert first.path is not None
    assert second.path is not None
    assert first.nodes == second.nodes
    assert first.iters == second.iters
    assert len(first.path) == len(second.path)
    assert np.allclose(first.path[-1], second.path[-1])


def test_rrt_planner_path_is_collision_free_and_reaches_goal() -> None:
    space = _build_space()
    start = np.array([1.0, 1.0, 1.0], dtype=float)
    goal_center = np.array([9.0, 9.0, 1.0], dtype=float)
    goal_tolerance = 0.75
    params = RrtParams(
        max_iters=2_000,
        step_size=0.6,
        goal_sample_rate=0.15,
        max_sample_tries=2_000,
        collision_step=0.1,
    )
    goal = GoalState(
        state=goal_center,
        radius=goal_tolerance,
        distance_fn=space.distance,
    )

    result = RrtPlanner(space, params, np.random.default_rng(11)).plan(start, goal)

    assert result.success
    assert result.path is not None
    assert np.allclose(result.path[0], start)
    assert space.distance(result.path[-1], goal_center) <= goal_tolerance

    for index in range(len(result.path) - 1):
        assert space.is_motion_valid(result.path[index], result.path[index + 1])
