"""Contract tests for the headless 3D ``RrtStarPlanner``."""

from __future__ import annotations

import numpy as np

from pathplanning.core.params import RrtParams
from pathplanning.planners.sampling.rrt_star import RrtStarPlanner
from pathplanning.spaces.continuous_3d import AABB, ContinuousSpace3D


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

    first = RrtStarPlanner(_build_space(), params, np.random.default_rng(7)).plan(
        start, (goal_center, goal_tolerance)
    )
    second = RrtStarPlanner(_build_space(), params, np.random.default_rng(7)).plan(
        start, (goal_center, goal_tolerance)
    )

    assert first.success and second.success
    assert first.path is not None
    assert second.path is not None
    assert first.nodes == second.nodes
    assert first.iters == second.iters
    assert len(first.path) == len(second.path)
    assert np.allclose(first.path[-1], second.path[-1])
    assert first.stats["path_cost"] == second.stats["path_cost"]


def test_rrtstar_planner_path_collision_free_and_cost_monotonic() -> None:
    space = _build_space()
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

    result = RrtStarPlanner(space, params, np.random.default_rng(11)).plan(
        start, (goal_center, goal_tolerance)
    )

    assert result.success
    assert result.path is not None
    assert np.allclose(result.path[0], start)
    assert space.distance(result.path[-1], goal_center) <= goal_tolerance

    cumulative_costs = [0.0]
    cumulative = 0.0
    for index in range(len(result.path) - 1):
        src = result.path[index]
        dst = result.path[index + 1]
        assert space.segment_free(src, dst, params.collision_step)
        edge_cost = space.distance(src, dst)
        cumulative += edge_cost
        cumulative_costs.append(cumulative)

    for previous, current in zip(cumulative_costs[:-1], cumulative_costs[1:], strict=True):
        assert current > previous
    assert np.isclose(cumulative_costs[-1], float(result.stats["path_cost"]))
