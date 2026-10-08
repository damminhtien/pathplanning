"""Riemannian informed tree search backed by the native C engine."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import fields
from typing import Any, cast

from pathplanning.core.contracts import ContinuousProblem, State
from pathplanning.core.params import RitParams, RrtParams
from pathplanning.core.results import PlanResult
from pathplanning.core.trace import TraceOptions
from pathplanning.core.types import RNG
from pathplanning.native.continuous import run_native_continuous
from pathplanning.planners.sampling._internal.continuous import (
    euclidean_distance,
    exact_goal_state,
    validate_objective,
)
from pathplanning.planners.sampling._internal.problem_adapter import resolve_rng

_ALLOWED_PARAM_KEYS = {field.name for field in fields(RitParams)} | {
    field.name for field in fields(RrtParams)
}
_IGNORED_RRT_KEYS = {
    "goal_reach_tolerance",
    "goal_sample_rate",
    "rrt_star_radius_gamma",
    "rrt_star_radius_max_factor",
    "rrt_star_radius_bias",
    "abit_inflation_parameter",
    "abit_truncation_parameter",
}


def _legacy_radius_factor(value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError("rrt_star_radius_max_factor must be a finite real number")
    return float(value)


def _coerce_params(
    problem: ContinuousProblem[State],
    params: RitParams | RrtParams | Mapping[str, object] | None,
) -> RitParams:
    values = dict(problem.params or {})
    if isinstance(params, RitParams):
        values.update(dict(params))
    elif isinstance(params, RrtParams):
        values.update(
            max_iters=params.max_iters,
            sample_count=params.sample_count,
            batch_size=params.batch_size,
            max_sample_tries=params.max_sample_tries,
            step_size=params.step_size,
            collision_step=params.collision_step,
            time_budget_s=params.time_budget_s,
            gamma=params.rrt_star_radius_gamma,
            max_connection_radius=(params.step_size * params.rrt_star_radius_max_factor),
            allow_python_callbacks=params.allow_python_callbacks,
        )
    elif params is not None:
        values.update(params)
    if "rrt_star_radius_gamma" in values:
        values.setdefault("gamma", values.pop("rrt_star_radius_gamma"))
    if "rrt_star_radius_max_factor" in values:
        radius_factor = values.pop("rrt_star_radius_max_factor")
        if "max_connection_radius" not in values:
            values["max_connection_radius"] = _legacy_radius_factor(
                values.get("step_size", 0.5)
            ) * _legacy_radius_factor(radius_factor)
    invalid = set(values) - _ALLOWED_PARAM_KEYS
    if invalid:
        invalid_values = ", ".join(sorted(invalid))
        raise KeyError(f"Unsupported RIT* params: {invalid_values}")
    for key in _IGNORED_RRT_KEYS:
        values.pop(key, None)
    return RitParams(**cast(Any, values)).validate()


def plan_rit_star(
    problem: ContinuousProblem[State],
    *,
    params: RitParams | RrtParams | Mapping[str, object] | None = None,
    rng: RNG | None = None,
    trace: TraceOptions | None = None,
) -> PlanResult:
    """Plan with RIT* under a constant or state-dependent SPD metric field."""
    validate_objective(problem.objective, "RIT*")
    resolved_params = _coerce_params(problem, params)
    goal = exact_goal_state(problem.goal, dim=len(problem.start))
    metric_tensor = getattr(problem.space, "metric_tensor", None)
    if callable(metric_tensor):
        bounds = getattr(problem.space, "metric_eigenvalue_bounds", None)
        if bounds is None:
            raise ValueError("metric_tensor spaces must provide global metric_eigenvalue_bounds")
        if not resolved_params.allow_python_callbacks:
            raise ValueError("custom metric tensors require RitParams(allow_python_callbacks=True)")
    else:
        euclidean_distance(problem.space, problem.start, goal)

    return run_native_continuous(
        problem.space,
        problem.start,
        problem.goal,
        resolved_params,
        resolve_rng(rng),
        planner="rit_star",
        trace=trace,
    )


__all__ = ["plan_rit_star"]
