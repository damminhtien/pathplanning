"""Focused checks for JIT* Jacobian scoring and just-in-time refinement."""

from __future__ import annotations

import numpy as np
import pytest

from pathplanning import JitParams, plan_continuous
from pathplanning.core.contracts import ContinuousProblem, GoalState
from pathplanning.spaces import AnisotropicContinuousSpace, PlanarManipulatorSpace


def test_jit_star_requires_a_robot_jacobian_when_scoring_manipulability() -> None:
    space = AnisotropicContinuousSpace([0.0], [1.0], [[1.0]])
    with pytest.raises(ValueError, match="requires space.jacobian"):
        plan_continuous(
            ContinuousProblem(space, [0.0], GoalState([1.0])),
            planner="jit_star",
            params=JitParams(sample_count=8, batch_size=4),
            seed=3,
        )


def test_jit_star_uses_jacobian_cost_and_collision_guidance() -> None:
    space = PlanarManipulatorSpace(
        [-2.4, -2.4, -2.4],
        [2.4, 2.4, 2.4],
        [1.0, 0.8, 0.6],
        obstacles=((2.0, 0.0, 0.15),),
    )
    start = np.asarray([-0.8, 0.5, 0.7])
    goal = np.asarray([0.8, -0.5, -0.7])
    result = plan_continuous(
        ContinuousProblem(space, start, GoalState(goal)),
        planner="jit_star",
        params=JitParams(
            sample_count=96,
            batch_size=32,
            max_iters=1_000,
            step_size=0.7,
            goal_sample_rate=0.25,
            gamma=10.0,
            max_connection_radius=3.0,
            jit_ancestor_depth=8,
            jit_sample_radius=0.3,
            jit_bias_probability=0.5,
            manipulability_weight=0.001,
            allow_python_callbacks=True,
        ),
        seed=2,
    )

    assert result.success
    assert result.path is not None
    assert np.allclose(result.path[0], start)
    assert np.allclose(result.path[-1], goal)
    assert result.stats["manipulability_evaluations"] > 0
    assert result.stats["jit_biased_samples"] > 0
    assert result.stats["jit_ancestor_candidates"] > 0
    assert result.stats["path_cost"] > float(np.linalg.norm(goal - start))
    assert all(
        space.is_motion_valid(first, second) for first, second in zip(result.path, result.path[1:])
    )
    assert space.jacobian(start).shape == (2, 3)

    folded = PlanarManipulatorSpace([-np.pi] * 3, [np.pi] * 3, [1.0, 1.0, 1.0])
    assert not folded.is_state_valid([0.0, np.pi, 0.0])
