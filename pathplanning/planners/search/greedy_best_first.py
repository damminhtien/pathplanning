"""Greedy best-first discrete-search planner."""

from __future__ import annotations

from collections.abc import Mapping
from typing import TypeVar

from pathplanning.core.contracts import DiscreteProblem
from pathplanning.core.results import PlanResult
from pathplanning.core.trace import TraceOptions
from pathplanning.core.types import RNG
from pathplanning.planners.search._internal.common import coerce_max_expansions
from pathplanning.planners.search._internal.native import run_native_greedy_best_first

N = TypeVar("N")


def plan_greedy_best_first(
    problem: DiscreteProblem[N],
    *,
    params: Mapping[str, object] | None = None,
    rng: RNG | None = None,
    trace: TraceOptions | None = None,
) -> PlanResult:
    """Plan a path for one ``DiscreteProblem`` with greedy best-first search."""
    _ = rng
    return run_native_greedy_best_first(
        problem,
        max_expansions=coerce_max_expansions(params),
        trace=trace,
    )


__all__ = ["plan_greedy_best_first"]
