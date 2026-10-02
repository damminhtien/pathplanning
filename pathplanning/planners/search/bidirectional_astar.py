"""Bidirectional A* over native CSR graphs."""

from __future__ import annotations

from collections.abc import Mapping
from typing import TypeVar

from pathplanning.core.contracts import DiscreteProblem
from pathplanning.core.results import PlanResult
from pathplanning.core.types import RNG
from pathplanning.planners.search._internal.common import coerce_max_expansions
from pathplanning.planners.search._internal.native import run_native_bidirectional_astar

N = TypeVar("N")


def plan_bidirectional_astar(
    problem: DiscreteProblem[N],
    *,
    params: Mapping[str, object] | None = None,
    rng: RNG | None = None,
) -> PlanResult:
    """Plan an optimal path using bidirectional A* and a consistent heuristic."""
    _ = rng
    if hasattr(problem.goal, "is_goal"):
        raise ValueError("bidirectional_astar requires an exact goal node")
    return run_native_bidirectional_astar(
        problem,
        max_expansions=coerce_max_expansions(params),
    )


__all__ = ["plan_bidirectional_astar"]
