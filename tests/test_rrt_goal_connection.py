"""Regression tests for RRT goal-connection behavior."""

from __future__ import annotations

import math

import numpy as np

from pathplanning.core.contracts import GoalState
from pathplanning.core.params import RrtParams
from pathplanning.planners.sampling.rrt import RrtPlanner


class OpenPlane2D:
    """Simple obstacle-free 2D space for RRT contract tests."""

    def __init__(self) -> None:
        self.lower = np.array([0.0, 0.0], dtype=float)
        self.upper = np.array([50.0, 30.0], dtype=float)

    def sample_free(self, rng: np.random.Generator) -> np.ndarray:
        return rng.uniform(self.lower, self.upper)

    def is_state_valid(self, x: np.ndarray) -> bool:
        point = np.asarray(x, dtype=float)
        if point.shape != (2,):
            return False
        return bool(np.all(point >= self.lower) and np.all(point <= self.upper))

    def is_motion_valid(self, a: np.ndarray, b: np.ndarray) -> bool:
        return self.is_state_valid(a) and self.is_state_valid(b)

    def distance(self, a: np.ndarray, b: np.ndarray) -> float:
        return float(np.linalg.norm(np.asarray(b, dtype=float) - np.asarray(a, dtype=float)))

    def steer(self, a: np.ndarray, b: np.ndarray, step_size: float) -> np.ndarray:
        if step_size <= 0.0:
            raise ValueError("step_size must be > 0")
        start = np.asarray(a, dtype=float)
        target = np.asarray(b, dtype=float)
        direction = target - start
        norm = float(np.linalg.norm(direction))
        if norm <= step_size:
            return target
        return start + (direction / norm) * float(step_size)


def test_rrt_goal_connection_returns_valid_path() -> None:
    rng = np.random.default_rng(0)
    start = np.array([2.0, 2.0], dtype=float)
    goal = np.array([4.0, 2.0], dtype=float)
    step_len = 1.0

    space = OpenPlane2D()
    planner = RrtPlanner(
        space=space,
        params=RrtParams(step_size=step_len, goal_sample_rate=1.0, max_iters=10),
        rng=rng,
    )
    goal_region = GoalState(state=goal, radius=1e-9, distance_fn=space.distance)
    result = planner.plan(start, goal_region)
    path = result.path

    assert result.success
    assert path is not None
    assert np.allclose(path[0], start)
    assert np.allclose(path[-1], goal)

    tolerance = 1e-9
    for idx in range(len(path) - 1):
        x1, y1 = path[idx]
        x2, y2 = path[idx + 1]
        segment_len = math.hypot(x2 - x1, y2 - y1)
        assert segment_len <= step_len + tolerance
