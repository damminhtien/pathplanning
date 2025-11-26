"""Weighted A* discrete-search planner."""

from __future__ import annotations

from collections.abc import Mapping
from typing import TypeVar

from pathplanning.core.contracts import DiscreteProblem
from pathplanning.core.results import PlanResult
from pathplanning.core.types import RNG
from pathplanning.planners.search._internal.common import (
    coerce_max_expansions,
    coerce_weight,
    run_best_first,
)

N = TypeVar("N")


def plan_weighted_astar(
    problem: DiscreteProblem[N],
    *,
    params: Mapping[str, object] | None = None,
    rng: RNG | None = None,
) -> PlanResult:
    """Plan a path for one ``DiscreteProblem`` with weighted A*."""
    _ = rng
    max_expansions = coerce_max_expansions(params)
    weight = coerce_weight(params)
    return run_best_first(
        problem,
        max_expansions=max_expansions,
        use_heuristic=True,
        heuristic_weight=weight,
    )


__all__ = ["plan_weighted_astar"]
