"""Small, stable public API for planner execution."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import replace
from typing import Any, TypeAlias, overload

import numpy as np

from pathplanning.core.contracts import (
    ContinuousProblem,
    DiscreteProblem,
    State,
    TemporalProblem,
)
from pathplanning.core.params import RrtParams
from pathplanning.core.results import PlanResult, TemporalPlanResult
from pathplanning.core.trace import TraceOptions
from pathplanning.core.types import RNG
from pathplanning.registry import (
    get_continuous_planner,
    get_discrete_planner,
    get_temporal_planner,
)

Result: TypeAlias = PlanResult
Stats: TypeAlias = Mapping[str, float]
DiscreteParams: TypeAlias = Mapping[str, object]
ContinuousParams: TypeAlias = RrtParams | Mapping[str, object]
TemporalParams: TypeAlias = Mapping[str, object]


def _resolve_rng(seed: int | None, rng: RNG | None) -> RNG:
    if rng is not None:
        return rng
    return np.random.default_rng(seed)


def plan_discrete(
    problem: DiscreteProblem[Any],
    *,
    planner: str = "astar",
    params: DiscreteParams | None = None,
    seed: int | None = 0,
    rng: RNG | None = None,
    trace: TraceOptions | None = None,
) -> Result:
    """Run one registered discrete planner on a ``DiscreteProblem``."""
    if trace is not None and type(trace) is not TraceOptions:
        raise TypeError("trace must be TraceOptions or None")
    effective_rng = _resolve_rng(seed, rng)
    merged_params = dict(problem.params or {})
    if params is not None:
        merged_params.update(params)
    resolved_params = merged_params or None
    resolved_problem = replace(problem, params=resolved_params)
    planner_fn = get_discrete_planner(planner)
    return planner_fn(resolved_problem, params=resolved_params, rng=effective_rng, trace=trace)


def plan_continuous(
    problem: ContinuousProblem[State],
    *,
    planner: str = "rrt_star",
    params: ContinuousParams | None = None,
    seed: int | None = 0,
    rng: RNG | None = None,
    trace: TraceOptions | None = None,
) -> Result:
    """Run one registered continuous planner on a ``ContinuousProblem``."""
    if trace is not None and type(trace) is not TraceOptions:
        raise TypeError("trace must be TraceOptions or None")
    effective_rng = _resolve_rng(seed, rng)
    resolved_params: RrtParams | dict[str, object] | None
    if params is None or isinstance(params, RrtParams):
        resolved_params = params
    else:
        resolved_params = dict(params)

    planner_fn = get_continuous_planner(planner)
    return planner_fn(problem, params=resolved_params, rng=effective_rng, trace=trace)


def plan_temporal(
    problem: TemporalProblem[Any],
    *,
    planner: str = "sipp",
    params: TemporalParams | None = None,
    seed: int | None = 0,
    rng: RNG | None = None,
    trace: TraceOptions | None = None,
) -> TemporalPlanResult[Any]:
    """Run a registered planner on a time-dependent graph problem."""
    if trace is not None and type(trace) is not TraceOptions:
        raise TypeError("trace must be TraceOptions or None")
    effective_rng = _resolve_rng(seed, rng)
    merged_params = dict(problem.params or {})
    if params is not None:
        merged_params.update(params)
    resolved_problem = replace(problem, params=merged_params or None)
    planner_fn = get_temporal_planner(planner)
    return planner_fn(
        resolved_problem, params=merged_params or None, rng=effective_rng, trace=trace
    )


@overload
def plan(
    problem: DiscreteProblem[Any],
    *,
    planner: str | None = None,
    params: DiscreteParams | None = None,
    seed: int | None = 0,
    rng: RNG | None = None,
    trace: TraceOptions | None = None,
) -> Result: ...


@overload
def plan(
    problem: ContinuousProblem[State],
    *,
    planner: str | None = None,
    params: ContinuousParams | None = None,
    seed: int | None = 0,
    rng: RNG | None = None,
    trace: TraceOptions | None = None,
) -> Result: ...


@overload
def plan(
    problem: TemporalProblem[Any],
    *,
    planner: str | None = None,
    params: TemporalParams | None = None,
    seed: int | None = 0,
    rng: RNG | None = None,
    trace: TraceOptions | None = None,
) -> TemporalPlanResult[Any]: ...


def plan(
    problem: DiscreteProblem[Any] | ContinuousProblem[State] | TemporalProblem[Any],
    *,
    planner: str | None = None,
    params: DiscreteParams | ContinuousParams | None = None,
    seed: int | None = 0,
    rng: RNG | None = None,
    trace: TraceOptions | None = None,
) -> Result:
    """Unified planner entrypoint dispatching by problem type."""
    if isinstance(problem, DiscreteProblem):
        resolved_planner = planner if planner is not None else "astar"
        if isinstance(params, RrtParams):
            raise TypeError("Discrete planning does not accept RrtParams")
        return plan_discrete(
            problem,
            planner=resolved_planner,
            params=params,
            seed=seed,
            rng=rng,
            trace=trace,
        )

    if isinstance(problem, TemporalProblem):
        if isinstance(params, RrtParams):
            raise TypeError("Temporal planning does not accept RrtParams")
        return plan_temporal(
            problem,
            planner=planner or "sipp",
            params=params,
            seed=seed,
            rng=rng,
            trace=trace,
        )

    resolved_planner = planner if planner is not None else "rrt_star"
    return plan_continuous(
        problem,
        planner=resolved_planner,
        params=params,
        seed=seed,
        rng=rng,
        trace=trace,
    )


__all__ = [
    "Result",
    "Stats",
    "plan_discrete",
    "plan_continuous",
    "plan_temporal",
    "plan",
]
