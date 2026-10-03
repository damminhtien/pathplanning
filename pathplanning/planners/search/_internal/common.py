"""Shared parameter utilities for discrete-search planners."""

from __future__ import annotations

from collections.abc import Mapping
import math
from typing import TypeVar

from pathplanning.core.contracts import DiscreteProblem
from pathplanning.core.results import PlanResult
from pathplanning.core.trace import TraceOptions
from pathplanning.planners.search._internal.native import run_native_best_first

N = TypeVar("N")


def coerce_max_expansions(params: Mapping[str, object] | None) -> int | None:
    """Parse and validate optional expansion budget from planner params."""
    if params is None:
        return None
    raw_value = params.get("max_expansions")
    if raw_value is None:
        return None
    if isinstance(raw_value, bool) or type(raw_value) is not int:
        raise TypeError("max_expansions must be an integer when provided")
    if raw_value <= 0:
        raise ValueError("max_expansions must be > 0 when provided")
    return raw_value


def coerce_weight(params: Mapping[str, object] | None) -> float:
    """Parse and validate weighted-A* heuristic weight."""
    if params is None:
        return 1.0
    raw_value = params.get("weight", 1.0)
    if isinstance(raw_value, bool) or not isinstance(raw_value, (int, float)):
        raise TypeError("weight must be a finite real number when provided")
    weight = float(raw_value)
    if not math.isfinite(weight):
        raise TypeError("weight must be a finite real number when provided")
    if weight < 1.0:
        raise ValueError("weight must be >= 1.0")
    return weight


def run_best_first(
    problem: DiscreteProblem[N],
    *,
    max_expansions: int | None,
    use_heuristic: bool,
    heuristic_weight: float = 1.0,
    trace: TraceOptions | None = None,
) -> PlanResult:
    """Run reusable best-first search through the native C++ core."""
    return run_native_best_first(
        problem,
        max_expansions=max_expansions,
        use_heuristic=use_heuristic,
        heuristic_weight=heuristic_weight,
        trace=trace,
    )


__all__ = [
    "coerce_max_expansions",
    "coerce_weight",
    "run_best_first",
]
