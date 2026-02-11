"""Regression tests for RRT goal-connection behavior."""

from __future__ import annotations

import math

import numpy as np

from pathplanning.Sampling_based_Planning.rrt_2D.rrt import Rrt


def test_rrt_goal_connection_returns_valid_path() -> None:
    np.random.seed(0)
    start = (2.0, 2.0)
    goal = (4.0, 2.0)
    step_len = 1.0

    planner = Rrt(
        s_start=start,
        s_goal=goal,
        step_len=step_len,
        goal_sample_rate=1.0,
        iter_max=10,
    )

    path = planner.planning()

    assert path is not None
    assert np.allclose(path[0], goal)
    assert np.allclose(path[-1], start)

    tolerance = 1e-9
    for idx in range(len(path) - 1):
        x1, y1 = path[idx]
        x2, y2 = path[idx + 1]
        segment_len = math.hypot(x2 - x1, y2 - y1)
        assert segment_len <= step_len + tolerance
