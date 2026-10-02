"""Correctness smoke coverage for the registered advanced sampling planners."""

from __future__ import annotations

import numpy as np
import pytest

from pathplanning.api import plan_continuous
from pathplanning.core.contracts import ContinuousProblem, GoalState


class _OpenLineSpace:
    def sample_free(self, rng: np.random.Generator) -> np.ndarray:
        return np.asarray([rng.random()], dtype=float)

    def is_state_valid(self, _state: np.ndarray) -> bool:
        return True

    def is_motion_valid(self, _start: np.ndarray, _end: np.ndarray) -> bool:
        return True

    def distance(self, start: np.ndarray, end: np.ndarray) -> float:
        return float(np.linalg.norm(np.asarray(end) - np.asarray(start)))

    def steer(self, start: np.ndarray, target: np.ndarray, step_size: float) -> np.ndarray:
        delta = np.asarray(target) - np.asarray(start)
        distance = float(np.linalg.norm(delta))
        if distance <= step_size:
            return np.asarray(target, dtype=float)
        return np.asarray(start, dtype=float) + delta * (step_size / distance)


class _ScaledMetricSpace(_OpenLineSpace):
    def distance(self, start: np.ndarray, end: np.ndarray) -> float:
        return 2.0 * super().distance(start, end)


def _problem(sample_count: int = 64, batch_size: int = 16) -> ContinuousProblem[np.ndarray]:
    return ContinuousProblem(
        space=_OpenLineSpace(),
        start=np.asarray([0.0]),
        goal=GoalState(np.asarray([1.0])),
        params={
            "max_iters": 1_000,
            "step_size": 0.25,
            "goal_sample_rate": 1.0,
            "sample_count": sample_count,
            "batch_size": batch_size,
            "allow_python_callbacks": True,
        },
    )


@pytest.mark.parametrize(
    ("planner", "sample_count", "batch_size"),
    [
        ("informed_rrt_star", 64, 16),
        ("fmt_star", 256, 16),
        ("bit_star", 64, 16),
        ("abit_star", 64, 16),
        ("rrt_connect", 64, 16),
    ],
)
def test_registered_advanced_sampling_planners_find_valid_paths(
    planner: str,
    sample_count: int,
    batch_size: int,
) -> None:
    result = plan_continuous(
        _problem(sample_count, batch_size),
        planner=planner,
        seed=13,
    )

    assert result.success
    assert result.path is not None
    assert np.allclose(result.path[0], [0.0])
    assert np.allclose(result.path[-1], [1.0])
    assert result.stats["path_cost"] == pytest.approx(1.0)


@pytest.mark.parametrize("planner", ["fmt_star", "bit_star", "abit_star", "informed_rrt_star"])
def test_optimal_sampling_planners_reject_custom_objectives(planner: str) -> None:
    problem = _problem()
    problem.objective = object()  # type: ignore[assignment]

    with pytest.raises(ValueError, match="custom objective"):
        plan_continuous(problem, planner=planner, seed=13)


@pytest.mark.parametrize("planner", ["fmt_star", "bit_star", "abit_star", "informed_rrt_star"])
def test_euclidean_sampling_planners_reject_other_metrics(planner: str) -> None:
    problem = _problem()
    problem.space = _ScaledMetricSpace()

    with pytest.raises(ValueError, match="Euclidean state-space distance"):
        plan_continuous(problem, planner=planner, seed=13)
