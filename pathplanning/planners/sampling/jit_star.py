"""Just-in-time informed tree search backed by the native C engine."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import fields
from typing import Any, cast

import numpy as np

from pathplanning.core.contracts import ContinuousProblem, State
from pathplanning.core.params import JitParams, RrtParams
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

_ALLOWED_PARAM_KEYS = {field.name for field in fields(JitParams)} | {
    field.name for field in fields(RrtParams)
}
_IGNORED_RRT_KEYS = {
    "rrt_star_radius_bias",
    "abit_inflation_parameter",
    "abit_truncation_parameter",
}


def _number(value: object, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError(f"{name} must be a finite real number")
    return float(value)


def _coerce_params(
    problem: ContinuousProblem[State],
    params: JitParams | RrtParams | Mapping[str, object] | None,
) -> JitParams:
    values = dict(problem.params or {})
    if isinstance(params, JitParams):
        values.update(dict(params))
    elif isinstance(params, RrtParams):
        values.update(
            max_iters=params.max_iters,
            sample_count=params.sample_count,
            batch_size=params.batch_size,
            max_sample_tries=params.max_sample_tries,
            step_size=params.step_size,
            goal_sample_rate=params.goal_sample_rate,
            collision_step=params.collision_step,
            goal_reach_tolerance=params.goal_reach_tolerance,
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
        factor = values.pop("rrt_star_radius_max_factor")
        if "max_connection_radius" not in values:
            values["max_connection_radius"] = _number(
                values.get("step_size", 0.5), "step_size"
            ) * _number(factor, "rrt_star_radius_max_factor")
    invalid = set(values) - _ALLOWED_PARAM_KEYS
    if invalid:
        invalid_values = ", ".join(sorted(invalid))
        raise KeyError(f"Unsupported JIT* params: {invalid_values}")
    for key in _IGNORED_RRT_KEYS:
        values.pop(key, None)
    return JitParams(**cast(Any, values)).validate()


def plan_jit_star(
    problem: ContinuousProblem[State],
    *,
    params: JitParams | RrtParams | Mapping[str, object] | None = None,
    rng: RNG | None = None,
    trace: TraceOptions | None = None,
) -> PlanResult:
    """Plan with JIT* edge adaptation and Jacobian-based motion scoring."""
    validate_objective(problem.objective, "JIT*")
    resolved_params = _coerce_params(problem, params)
    goal = exact_goal_state(problem.goal, dim=len(problem.start))
    euclidean_distance(problem.space, problem.start, goal)

    if resolved_params.manipulability_weight > 0:
        jacobian = getattr(problem.space, "jacobian", None)
        if not callable(jacobian):
            raise ValueError(
                "JIT* manipulability scoring requires space.jacobian(state); "
                "set manipulability_weight=0 to disable it"
            )
        if not resolved_params.allow_python_callbacks:
            raise ValueError(
                "JIT* Jacobian callbacks require JitParams(allow_python_callbacks=True)"
            )
        matrix = np.asarray(jacobian(np.asarray(problem.start, dtype=np.float64)), dtype=np.float64)
        if matrix.ndim != 2 or matrix.shape[0] == 0 or matrix.shape[1] != len(problem.start):
            raise ValueError(
                "space.jacobian(state) must be a non-empty matrix with one column per joint"
            )
        if not np.all(np.isfinite(matrix)):
            raise ValueError("space.jacobian(state) must contain only finite values")

    return run_native_continuous(
        problem.space,
        problem.start,
        problem.goal,
        resolved_params,
        resolve_rng(rng),
        planner="jit_star",
        trace=trace,
    )


__all__ = ["plan_jit_star"]
