"""Small, stable public API for problem-oriented planner execution."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
import importlib
from types import ModuleType
from typing import Any, TypeAlias, overload

import numpy as np

from pathplanning.core.contracts import ContinuousProblem, ContinuousSpace, DiscreteProblem, GoalRegion, State
from pathplanning.core.params import RrtParams
from pathplanning.core.results import PlanResult
from pathplanning.core.types import RNG
from pathplanning.registry import (
    get_algorithm,
    planner_for_algorithm,
    run_registered_continuous_planner,
    run_registered_discrete_planner,
)

Result: TypeAlias = PlanResult
Stats: TypeAlias = Mapping[str, float]
DiscreteParams: TypeAlias = Mapping[str, object]
ContinuousParams: TypeAlias = RrtParams | Mapping[str, object]


def _resolve_rng(seed: int | None, rng: RNG | None) -> RNG:
    if rng is not None:
        return rng
    return np.random.default_rng(seed)


def load_algorithm_module(algorithm_id: str) -> ModuleType:
    """Load one planner module by legacy registry algorithm id."""
    return importlib.import_module(get_algorithm(algorithm_id).module)


def run_planner(
    algorithm_id: str,
    *,
    space: ContinuousSpace[State],
    start: Sequence[float] | State,
    goal: GoalRegion[State],
    params: RrtParams | None = None,
    seed: int | None = 0,
    rng: RNG | None = None,
) -> Result:
    """Compatibility wrapper for legacy ``algorithm_id`` based planner execution."""
    problem_kind, planner = planner_for_algorithm(algorithm_id)
    if problem_kind != "continuous":
        raise TypeError(f"Legacy algorithm '{algorithm_id}' does not map to a continuous planner")

    problem = ContinuousProblem(space=space, start=np.asarray(start, dtype=float), goal=goal)
    return plan_continuous(
        problem,
        planner=planner,
        params=params,
        seed=seed,
        rng=rng,
    )


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
    return run_registered_discrete_planner(
        planner,
        problem,
        params=resolved_params,
        rng=effective_rng,
    )


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

    return run_registered_continuous_planner(
        planner,
        problem,
        params=resolved_params,
        rng=effective_rng,
    )


@overload
def plan(
    problem: str,
    *,
    space: ContinuousSpace[State],
    start: Sequence[float] | State,
    goal: GoalRegion[State],
    planner: str | None = None,
    params: RrtParams | None = None,
    seed: int | None = 0,
    rng: RNG | None = None,
) -> Result: ...


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
    problem: str | DiscreteProblem[Any] | ContinuousProblem[State],
    *,
    space: ContinuousSpace[State] | None = None,
    start: Sequence[float] | State | None = None,
    goal: GoalRegion[State] | None = None,
    planner: str | None = None,
    params: DiscreteParams | ContinuousParams | None = None,
    seed: int | None = 0,
    rng: RNG | None = None,
) -> Result:
    """Unified planner entrypoint dispatching by problem type."""
    if isinstance(problem, str):
        if space is None or start is None or goal is None:
            raise TypeError(
                "Legacy plan(algorithm_id=...) requires space=..., start=..., and goal=..."
            )
        if planner is not None:
            raise TypeError(
                "Legacy plan(algorithm_id=...) does not accept planner=...; "
                "use run_planner() or plan_continuous() for explicit planner selection"
            )
        if params is not None and not isinstance(params, RrtParams):
            raise TypeError("Legacy plan(algorithm_id=...) expects params as RrtParams or None")
        return run_planner(
            problem,
            space=space,
            start=start,
            goal=goal,
            params=params,
            seed=seed,
            rng=rng,
        )

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
    "GoalRegion",
    "Result",
    "Stats",
    "load_algorithm_module",
    "run_planner",
    "plan_discrete",
    "plan_continuous",
    "plan",
]
