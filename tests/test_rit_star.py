"""Focused checks for RIT* metric costs and collision-adaptive refinement."""

from __future__ import annotations

import numpy as np
import pytest

from pathplanning import RitParams, plan_continuous
from pathplanning.core.contracts import ContinuousProblem, GoalState
from pathplanning.spaces import AnisotropicContinuousSpace


def test_rit_star_uses_constant_anisotropic_metric_for_path_cost() -> None:
    space = AnisotropicContinuousSpace([0.0], [1.0], [[4.0]])
    result = plan_continuous(
        ContinuousProblem(space, [0.0], GoalState([1.0])),
        planner="rit_star",
        params=RitParams(
            sample_count=32,
            batch_size=8,
            max_iters=2_000,
            allow_python_callbacks=True,
        ),
        seed=4,
    )

    assert result.success
    assert result.path is not None
    assert np.allclose(result.path[0], [0.0])
    assert np.allclose(result.path[-1], [1.0])
    assert result.stats["path_cost"] == pytest.approx(2.0)
    assert result.stats["metric_evaluations"] > 0


def test_rit_star_refines_metric_from_collision_feedback() -> None:
    space = AnisotropicContinuousSpace(
        [0.0, 0.0],
        [5.0, 5.0],
        np.eye(2),
        obstacles=(((2.0, 0.0), (3.0, 4.0)),),
        collision_step=0.1,
    )
    result = plan_continuous(
        ContinuousProblem(space, [0.5, 2.5], GoalState([4.5, 2.5])),
        planner="rit_star",
        params=RitParams(
            sample_count=128,
            batch_size=16,
            max_iters=20_000,
            gamma=6.0,
            max_connection_radius=3.0,
            collision_step=0.1,
            carm_sigma=0.3,
            carm_alpha=2.0,
            allow_python_callbacks=True,
        ),
        seed=13,
    )

    assert result.success
    assert result.path is not None
    assert result.stats["metric_updates"] > 0
    assert all(
        space.is_motion_valid(start, end) for start, end in zip(result.path, result.path[1:])
    )


def test_rit_star_requires_explicit_opt_in_for_python_metric_callbacks() -> None:
    space = AnisotropicContinuousSpace([0.0], [1.0], [[4.0]])
    with pytest.raises(ValueError, match="allow_python_callbacks=True"):
        plan_continuous(
            ContinuousProblem(space, [0.0], GoalState([1.0])),
            planner="rit_star",
            params=RitParams(sample_count=8, batch_size=4),
            seed=4,
        )


def test_rit_star_rejects_non_positive_definite_metric_callbacks() -> None:
    class InvalidMetricSpace(AnisotropicContinuousSpace):
        def metric_tensor(self, state: np.ndarray) -> np.ndarray:
            del state
            return np.asarray([[-1.0]])

    space = InvalidMetricSpace([0.0], [1.0], [[1.0]])
    with pytest.raises(RuntimeError, match="distance callback failed"):
        plan_continuous(
            ContinuousProblem(space, [0.0], GoalState([1.0])),
            planner="rit_star",
            params=RitParams(
                sample_count=8,
                batch_size=4,
                allow_python_callbacks=True,
            ),
            seed=4,
        )
