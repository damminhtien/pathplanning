"""Weighted A* with conditional re-expansion of closed nodes."""

from __future__ import annotations

from collections.abc import Mapping
import math
from typing import TypeVar

from pathplanning.core.contracts import DiscreteProblem
from pathplanning.core.results import PlanResult
from pathplanning.core.trace import TraceOptions
from pathplanning.core.types import RNG
from pathplanning.planners.search._internal.common import (
    coerce_max_expansions,
    coerce_weight,
    run_reexp_astar,
)

N = TypeVar("N")

_REOPEN_MODES = {"abs": 0, "rel_edge": 1, "rel_g": 2}
_TIE_BREAKS = {"g_low": 0, "g_high": 1}


def _coerce_threshold(params: Mapping[str, object] | None) -> float:
    """Parse the non-negative conditional-reopen threshold, allowing infinity."""
    if params is None:
        return 0.0
    raw_value = params.get("r", 0.0)
    if isinstance(raw_value, bool) or not isinstance(raw_value, (int, float)):
        raise TypeError("r must be a non-negative real number when provided")
    threshold = float(raw_value)
    if math.isnan(threshold) or threshold < 0.0:
        raise ValueError("r must be >= 0.0 or infinity")
    return threshold


def _coerce_mode(
    params: Mapping[str, object] | None,
    *,
    name: str,
    supported: Mapping[str, int],
    default: str,
) -> int:
    """Validate and encode one named ReExpAstar option."""
    raw_value = default if params is None else params.get(name, default)
    if not isinstance(raw_value, str):
        raise TypeError(f"{name} must be a string when provided")
    try:
        return supported[raw_value]
    except KeyError as exc:
        choices = ", ".join(repr(value) for value in supported)
        raise ValueError(f"{name} must be one of {choices}") from exc


def _coerce_max_runtime_ms(params: Mapping[str, object] | None) -> float:
    """Parse an optional non-negative search-time budget in milliseconds."""
    if params is None:
        return 0.0
    raw_value = params.get("max_runtime_ms")
    if raw_value is None:
        return 0.0
    if isinstance(raw_value, bool) or not isinstance(raw_value, (int, float)):
        raise TypeError("max_runtime_ms must be a finite non-negative real number")
    budget = float(raw_value)
    if not math.isfinite(budget) or budget < 0.0:
        raise ValueError("max_runtime_ms must be finite and >= 0.0")
    return budget


def plan_reexp_astar(
    problem: DiscreteProblem[N],
    *,
    params: Mapping[str, object] | None = None,
    rng: RNG | None = None,
    trace: TraceOptions | None = None,
) -> PlanResult:
    """Plan a discrete path with Weighted A* and conditional re-expansion.

    Parameters use the ReExpAstar names: ``weight`` inflates the heuristic;
    ``r`` controls reopening using ``r_mode``; ``tie_break`` selects lower or
    higher ``g`` values when ``f`` ties. ``r=0`` always reopens an improved
    closed node, while ``r=inf`` never reopens one.
    """
    _ = rng
    return run_reexp_astar(
        problem,
        max_expansions=coerce_max_expansions(params),
        max_runtime_ms=_coerce_max_runtime_ms(params),
        weight=coerce_weight(params),
        threshold=_coerce_threshold(params),
        reopen_mode=_coerce_mode(
            params,
            name="r_mode",
            supported=_REOPEN_MODES,
            default="abs",
        ),
        tie_break=_coerce_mode(
            params,
            name="tie_break",
            supported=_TIE_BREAKS,
            default="g_low",
        ),
        trace=trace,
    )


__all__ = ["plan_reexp_astar"]
