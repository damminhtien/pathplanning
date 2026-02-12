"""Core contract and parameter validation tests."""

from __future__ import annotations

import pytest

from pathplanning.core.contracts import BatchConfigurationSpace, ConfigurationSpace
from pathplanning.core.params import RrtParams
from pathplanning.core.results import PlanResult, StopReason


def test_core_contracts_importable() -> None:
    """Core contracts should be importable and expose typed result models."""
    assert ConfigurationSpace is not None
    assert BatchConfigurationSpace is not None

    result = PlanResult(
        success=True,
        path=None,
        best_path=None,
        stop_reason=StopReason.SUCCESS,
        iters=0,
        nodes=0,
    )
    assert result.success is True
    assert result.path is None
    assert result.best_path is None
    assert result.stop_reason is StopReason.SUCCESS
    assert result.iters == 0
    assert result.nodes == 0
    assert result.stats == {}


def test_rrt_params_validate_accepts_valid_values() -> None:
    """A valid parameter set should pass strict validation."""
    params = RrtParams(
        max_iters=500,
        step_size=0.25,
        goal_sample_rate=0.2,
        time_budget_s=2.0,
        max_sample_tries=20,
        collision_step=0.05,
        goal_reach_tolerance=1e-6,
        rrt_star_radius_gamma=1.5,
        rrt_star_radius_bias=0.5,
        rrt_star_radius_max_factor=3.0,
    )
    assert params.validate() is params


@pytest.mark.parametrize(
    ("kwargs", "error_type"),
    [
        ({"max_iters": 0}, ValueError),
        ({"max_iters": 1.5}, TypeError),
        ({"step_size": 0.0}, ValueError),
        ({"step_size": "bad"}, TypeError),
        ({"goal_sample_rate": -0.01}, ValueError),
        ({"goal_sample_rate": 1.01}, ValueError),
        ({"goal_sample_rate": "bad"}, TypeError),
        ({"time_budget_s": 0.0}, ValueError),
        ({"time_budget_s": "bad"}, TypeError),
        ({"max_sample_tries": 0}, ValueError),
        ({"max_sample_tries": 1.2}, TypeError),
        ({"collision_step": 0.0}, ValueError),
        ({"collision_step": "bad"}, TypeError),
        ({"goal_reach_tolerance": -0.1}, ValueError),
        ({"goal_reach_tolerance": "bad"}, TypeError),
        ({"rrt_star_radius_gamma": 0.0}, ValueError),
        ({"rrt_star_radius_gamma": "bad"}, TypeError),
        ({"rrt_star_radius_bias": -0.1}, ValueError),
        ({"rrt_star_radius_bias": "bad"}, TypeError),
        ({"rrt_star_radius_max_factor": 0.0}, ValueError),
        ({"rrt_star_radius_max_factor": "bad"}, TypeError),
    ],
)
def test_rrt_params_validate_rejects_invalid_values(
    kwargs: dict[str, object], error_type: type[Exception]
) -> None:
    """Invalid parameter values should fail fast with strict checks."""
    with pytest.raises(error_type):
        RrtParams(**kwargs)
