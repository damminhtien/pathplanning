"""A* discrete-search planner."""

from __future__ import annotations

from collections.abc import Mapping
from typing import TypeVar

from pathplanning.core.contracts import DiscreteProblem
from pathplanning.core.results import PlanResult
from pathplanning.core.trace import TraceOptions
from pathplanning.core.types import RNG
from pathplanning.planners.search._internal.common import coerce_max_expansions, run_best_first

N = TypeVar("N")


def plan_astar(
    problem: DiscreteProblem[N],
    *,
    params: Mapping[str, object] | None = None,
    rng: RNG | None = None,
    trace: TraceOptions | None = None,
) -> PlanResult:
    """Plan a path for one ``DiscreteProblem`` with A*."""
    _ = rng
    max_expansions = coerce_max_expansions(params)
    return run_best_first(
        problem,
        max_expansions=max_expansions,
        use_heuristic=True,
        trace=trace,
    )


__all__ = ["plan_astar"]
