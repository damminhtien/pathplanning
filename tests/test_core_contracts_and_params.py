"""Core contract and parameter validation tests."""

from __future__ import annotations

import pytest

from pathplanning.core.contracts import ConfigurationSpace, PlanResult
from pathplanning.core.params import RrtParams


def test_core_contracts_importable() -> None:
    """Core contracts should be importable and have usable defaults."""
    assert ConfigurationSpace is not None

    result = PlanResult(success=True)
    assert result.success is True
    assert result.path == []
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
    ],
)
def test_rrt_params_validate_rejects_invalid_values(
    kwargs: dict[str, object], error_type: type[Exception]
) -> None:
    """Invalid parameter values should fail fast with strict checks."""
    with pytest.raises(error_type):
        RrtParams(**kwargs)
