"""Small, stable public API for planner execution."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeAlias, overload

import numpy as np

from pathplanning.core.contracts import ContinuousProblem, DiscreteProblem, State
from pathplanning.core.params import RrtParams
from pathplanning.core.results import PlanResult
from pathplanning.core.types import RNG
from pathplanning.registry import get_continuous_planner, get_discrete_planner

Result: TypeAlias = PlanResult
Stats: TypeAlias = Mapping[str, float]
DiscreteParams: TypeAlias = Mapping[str, object]
ContinuousParams: TypeAlias = RrtParams | Mapping[str, object]


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
) -> Result:
    """Run one registered discrete planner on a ``DiscreteProblem``."""
    effective_rng = _resolve_rng(seed, rng)
    resolved_params = dict(params) if params is not None else None
    planner_fn = get_discrete_planner(planner)
    return planner_fn(problem, params=resolved_params, rng=effective_rng)


def plan_continuous(
    problem: ContinuousProblem[State],
    *,
    planner: str = "rrt_star",
    params: ContinuousParams | None = None,
    seed: int | None = 0,
    rng: RNG | None = None,
) -> Result:
    """Run one registered continuous planner on a ``ContinuousProblem``."""
    effective_rng = _resolve_rng(seed, rng)
    resolved_params: RrtParams | dict[str, object] | None
    if params is None or isinstance(params, RrtParams):
        resolved_params = params
    else:
        resolved_params = dict(params)

    planner_fn = get_continuous_planner(planner)
    return planner_fn(problem, params=resolved_params, rng=effective_rng)


@overload
def plan(
    problem: DiscreteProblem[Any],
    *,
    planner: str | None = None,
    params: DiscreteParams | None = None,
    seed: int | None = 0,
    rng: RNG | None = None,
) -> Result: ...


@overload
def plan(
    problem: ContinuousProblem[State],
    *,
    planner: str | None = None,
    params: ContinuousParams | None = None,
    seed: int | None = 0,
    rng: RNG | None = None,
) -> Result: ...


def plan(
    problem: DiscreteProblem[Any] | ContinuousProblem[State],
    *,
    planner: str | None = None,
    params: DiscreteParams | ContinuousParams | None = None,
    seed: int | None = 0,
    rng: RNG | None = None,
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
        )

    resolved_planner = planner if planner is not None else "rrt_star"
    return plan_continuous(
        problem,
        planner=resolved_planner,
        params=params,
        seed=seed,
        rng=rng,
    )


__all__ = [
    "Result",
    "Stats",
    "plan_discrete",
    "plan_continuous",
    "plan",
]
