"""Bidirectional A* discrete-search planner."""

from __future__ import annotations

from collections.abc import Mapping
from typing import TypeVar, cast

from pathplanning.core.contracts import DiscreteProblem
from pathplanning.core.results import PlanResult
from pathplanning.core.types import RNG
from pathplanning.planners.search._internal.common import coerce_max_expansions
from pathplanning.planners.search._internal.native import run_native_bidirectional_astar

N = TypeVar("N")


def _coerce_exact_goal(problem: DiscreteProblem[N]) -> N | None:
    goal_value = problem.goal
    if hasattr(goal_value, "is_goal"):
        return None
    return cast(N, goal_value)


def plan_bidirectional_astar(
    problem: DiscreteProblem[N],
    *,
    params: Mapping[str, object] | None = None,
    rng: RNG | None = None,
) -> PlanResult:
    """Plan a path for one ``DiscreteProblem`` with bidirectional A*."""
    _ = rng
    if _coerce_exact_goal(problem) is None:
        from pathplanning.planners.search.astar import plan_astar

        return plan_astar(problem, params=params, rng=rng)
    return run_native_bidirectional_astar(
        problem,
        max_expansions=coerce_max_expansions(params),
    )


__all__ = ["plan_bidirectional_astar"]
