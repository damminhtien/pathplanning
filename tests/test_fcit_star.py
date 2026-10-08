"""Focused FCIT* checks for complete connectivity and batched edge validation."""

from __future__ import annotations

import numpy as np

from pathplanning import RrtParams, TraceOptions, plan_continuous
from pathplanning.core.contracts import ContinuousProblem, GoalState
from pathplanning.spaces.grid2d import Grid2DSamplingSpace


class _BatchSpace:
    distance_metric = "euclidean"

    def __init__(self) -> None:
        self.batch_sizes: list[int] = []

    def sample_free(self, rng: np.random.Generator) -> np.ndarray:
        return rng.uniform(0.0, 1.0, size=2)

    def is_state_valid(self, state: np.ndarray) -> bool:
        return bool(np.all((state >= 0.0) & (state <= 1.0)))

    def is_motion_valid(self, start: np.ndarray, end: np.ndarray) -> bool:
        return not (np.allclose(start, (0.1, 0.1)) and np.allclose(end, (0.9, 0.9)))

    def is_motion_valid_batch_with_step(
        self, edges: list[tuple[np.ndarray, np.ndarray]], collision_step: float
    ) -> list[bool]:
        del collision_step
        self.batch_sizes.append(len(edges))
        return [self.is_motion_valid(start, end) for start, end in edges]

    def distance(self, start: np.ndarray, end: np.ndarray) -> float:
        return float(np.linalg.norm(end - start))

    def steer(self, start: np.ndarray, end: np.ndarray, step_size: float) -> np.ndarray:
        delta = end - start
        distance = self.distance(start, end)
        return end.copy() if distance <= step_size else start + delta * (step_size / distance)


def test_fcit_star_searches_complete_graph_and_batches_motion_checks() -> None:
    space = _BatchSpace()
    start = np.asarray((0.1, 0.1))
    goal = np.asarray((0.9, 0.9))
    result = plan_continuous(
        ContinuousProblem(space, start, GoalState(goal)),
        planner="fcit_star",
        params=RrtParams(
            sample_count=24,
            batch_size=8,
            max_iters=512,
            allow_python_callbacks=True,
        ),
        seed=17,
        trace=TraceOptions(16_384),
    )

    assert result.success
    assert result.path is not None and result.path.shape[0] >= 3
    assert np.allclose(result.path[0], start) and np.allclose(result.path[-1], goal)
    assert space.batch_sizes and max(space.batch_sizes) > 1
    assert result.stats["motion_checks"] == sum(space.batch_sizes)
    assert result.trace is not None and result.trace.events.size > 0


def test_fcit_star_uses_native_scalar_motion_fallback() -> None:
    space = Grid2DSamplingSpace(x_range=(0.0, 2.0), y_range=(0.0, 2.0))
    result = plan_continuous(
        ContinuousProblem(space, (0.25, 0.25), GoalState((1.75, 1.75))),
        planner="fcit_star",
        params=RrtParams(sample_count=8, batch_size=4, max_iters=64),
        seed=5,
    )

    assert result.success
    assert result.path is not None and result.path.shape[0] == 2
    assert result.stats["motion_checks"] > 0
