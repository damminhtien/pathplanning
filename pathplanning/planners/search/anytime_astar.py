"""Anytime A* discrete-search planner."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
import math
from typing import TypeVar

from pathplanning.core.contracts import DiscreteProblem
from pathplanning.core.results import PlanResult, StopReason
from pathplanning.core.types import RNG
from pathplanning.planners.search.weighted_astar import plan_weighted_astar

N = TypeVar("N")

_DEFAULT_WEIGHTS: tuple[float, ...] = (2.5, 2.0, 1.5, 1.0)


def _coerce_anytime_weights(params: Mapping[str, object] | None) -> tuple[float, ...]:
    if params is None or "anytime_weights" not in params:
        return _DEFAULT_WEIGHTS

    raw = params["anytime_weights"]
    if isinstance(raw, (str, bytes)) or not isinstance(raw, Sequence):
        raise TypeError("anytime_weights must be a sequence of finite real values")

    weights: list[float] = []
    for value in raw:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise TypeError("anytime_weights must be a sequence of finite real values")
        weight = float(value)
        if not math.isfinite(weight):
            raise TypeError("anytime_weights must be a sequence of finite real values")
        if weight < 1.0:
            raise ValueError("anytime_weights values must be >= 1.0")
        weights.append(weight)

    if not weights:
        raise ValueError("anytime_weights must be non-empty")
    return tuple(weights)


def plan_anytime_astar(
    problem: DiscreteProblem[N],
    *,
    params: Mapping[str, object] | None = None,
    rng: RNG | None = None,
) -> PlanResult:
    """Plan a path for one ``DiscreteProblem`` with Anytime A*."""
    weights = _coerce_anytime_weights(params)

    base_params = dict(params) if params is not None else {}
    base_params.pop("weight", None)
    base_params.pop("anytime_weights", None)

    total_iters = 0
    total_nodes = 0
    best_result: PlanResult | None = None
    best_cost = float("inf")

    for weight in weights:
        candidate_params = dict(base_params)
        candidate_params["weight"] = weight
        result = plan_weighted_astar(problem, params=candidate_params, rng=rng)
        total_iters += result.iters
        total_nodes += result.nodes

        if result.success and result.path is not None:
            cost = float(result.stats.get("path_cost", float("inf")))
            if cost < best_cost:
                best_cost = cost
                best_result = result
            if weight <= 1.0:
                break

    if best_result is None:
        return PlanResult(
            success=False,
            path=None,
            best_path=None,
            stop_reason=StopReason.NO_PROGRESS,
            iters=total_iters,
            nodes=total_nodes,
            stats={"attempts": float(len(weights))},
        )

    stats = dict(best_result.stats)
    stats["attempts"] = float(len(weights))
    return PlanResult(
        success=True,
        path=best_result.path,
        best_path=best_result.best_path,
        stop_reason=best_result.stop_reason,
        iters=total_iters,
        nodes=total_nodes,
        stats=stats,
    )


__all__ = ["plan_anytime_astar"]
