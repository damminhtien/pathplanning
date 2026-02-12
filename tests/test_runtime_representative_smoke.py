"""Representative runtime smoke tests across planning families."""

from __future__ import annotations

import numpy as np

from pathplanning.core.contracts import GoalState
from pathplanning.core.params import RrtParams
from pathplanning.planners.sampling.rrt import RrtPlanner
from pathplanning.planners.search.astar_3d import Weighted_A_star


class OpenPlane2D:
    """Simple obstacle-free 2D space for sampling runtime smoke tests."""

    def __init__(self) -> None:
        self.lower = np.array([0.0, 0.0], dtype=float)
        self.upper = np.array([60.0, 30.0], dtype=float)

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


def test_sampling2d_rrt_runtime_smoke() -> None:
    planner = RrtPlanner(
        space=OpenPlane2D(),
        params=RrtParams(step_size=0.5, goal_sample_rate=0.05, max_iters=200),
        rng=np.random.default_rng(0),
    )
    goal_region = GoalState(
        state=np.array([49.0, 24.0], dtype=float),
        radius=0.25,
        distance_fn=planner.space.distance,
    )
    result = planner.plan(np.array([2.0, 2.0], dtype=float), goal_region)
    assert result.path is None or isinstance(result.path, np.ndarray)


def test_search3d_astar_runtime_smoke() -> None:
    planner = Weighted_A_star(resolution=1.0)
    # Limit expansions to keep test fast and headless.
    ok = planner.run(N=200)
    assert ok is True
