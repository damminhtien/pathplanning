"""Problem-oriented continuous planner entrypoints."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import fields
from typing import Any

import numpy as np

from pathplanning.core.contracts import ContinuousProblem, State
from pathplanning.core.params import RrtParams
from pathplanning.core.results import PlanResult
from pathplanning.core.types import RNG
from pathplanning.planners.sampling.rrt import RrtPlanner
from pathplanning.planners.sampling.rrt_star import RrtStarPlanner

_ALLOWED_RRT_PARAM_KEYS = {field.name for field in fields(RrtParams)}


def _coerce_rrt_params(
    problem: ContinuousProblem[State],
    params: RrtParams | Mapping[str, object] | None,
) -> RrtParams:
    if isinstance(params, RrtParams):
        return params.validate()

    problem_params = problem.params
    merged_kwargs: dict[str, object] = {}

    if problem_params is not None:
        invalid = set(problem_params) - _ALLOWED_RRT_PARAM_KEYS
        if invalid:
            invalid_values = ", ".join(sorted(invalid))
            raise KeyError(f"Unsupported continuous problem params: {invalid_values}")
        merged_kwargs.update(problem_params)

    if params is not None:
        invalid = set(params) - _ALLOWED_RRT_PARAM_KEYS
        if invalid:
            invalid_values = ", ".join(sorted(invalid))
            raise KeyError(f"Unsupported planner params override: {invalid_values}")
        merged_kwargs.update(params)

    return RrtParams(**merged_kwargs)


def _resolve_rng(rng: RNG | None) -> RNG:
    return rng if rng is not None else np.random.default_rng(0)


def plan_rrt(
    problem: ContinuousProblem[State],
    *,
    params: RrtParams | Mapping[str, object] | None = None,
    rng: RNG | None = None,
) -> PlanResult:
    """Plan one ``ContinuousProblem`` with RRT."""
    resolved_params = _coerce_rrt_params(problem, params)
    planner = RrtPlanner(problem.space, resolved_params, _resolve_rng(rng))
    return planner.plan(problem.start, problem.goal)


def plan_rrt_star(
    problem: ContinuousProblem[State],
    *,
    params: RrtParams | Mapping[str, object] | None = None,
    rng: RNG | None = None,
) -> PlanResult:
    """Plan one ``ContinuousProblem`` with RRT*."""
    resolved_params = _coerce_rrt_params(problem, params)
    planner = RrtStarPlanner(problem.space, resolved_params, _resolve_rng(rng))
    return planner.plan(problem.start, problem.goal)


def plan_informed_rrt_star(
    problem: ContinuousProblem[State],
    *,
    params: RrtParams | Mapping[str, object] | None = None,
    rng: RNG | None = None,
) -> PlanResult:
    """Alias entrypoint currently backed by the RRT* core implementation."""
    return plan_rrt_star(problem, params=params, rng=rng)


def plan_bit_star(
    problem: ContinuousProblem[State],
    *,
    params: RrtParams | Mapping[str, object] | None = None,
    rng: RNG | None = None,
) -> PlanResult:
    """Alias entrypoint currently backed by the RRT* core implementation."""
    return plan_rrt_star(problem, params=params, rng=rng)


def plan_fmt_star(
    problem: ContinuousProblem[State],
    *,
    params: RrtParams | Mapping[str, object] | None = None,
    rng: RNG | None = None,
) -> PlanResult:
    """Alias entrypoint currently backed by the RRT core implementation."""
    return plan_rrt(problem, params=params, rng=rng)


def plan_rrt_connect(
    problem: ContinuousProblem[State],
    *,
    params: RrtParams | Mapping[str, object] | None = None,
    rng: RNG | None = None,
) -> PlanResult:
    """Alias entrypoint currently backed by the RRT core implementation."""
    return plan_rrt(problem, params=params, rng=rng)


def plan_abit_star(
    problem: ContinuousProblem[State],
    *,
    params: RrtParams | Mapping[str, object] | None = None,
    rng: RNG | None = None,
) -> PlanResult:
    """Alias entrypoint currently backed by the RRT* core implementation."""
    return plan_rrt_star(problem, params=params, rng=rng)


__all__ = [
    "plan_rrt",
    "plan_rrt_star",
    "plan_informed_rrt_star",
    "plan_bit_star",
    "plan_fmt_star",
    "plan_rrt_connect",
    "plan_abit_star",
]
