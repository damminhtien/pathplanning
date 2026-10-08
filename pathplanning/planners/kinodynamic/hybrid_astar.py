"""Hybrid A* for Ackermann-like SE(2) vehicles."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import fields
from typing import Any, cast

from pathplanning.core.contracts import ContinuousProblem, KinematicSpace, State
from pathplanning.core.params import HybridAStarParams, JitParams, RitParams, RrtParams
from pathplanning.core.results import KinematicPlanResult
from pathplanning.core.trace import TraceOptions
from pathplanning.core.types import RNG
from pathplanning.native.kinodynamic import run_hybrid_astar
from pathplanning.planners.sampling._internal.continuous import exact_goal_state

_PARAM_KEYS = {field.name for field in fields(HybridAStarParams)}


def _coerce_params(
    problem: ContinuousProblem[State],
    params: HybridAStarParams | JitParams | RitParams | RrtParams | Mapping[str, object] | None,
) -> HybridAStarParams:
    values = dict(problem.params or {})
    if isinstance(params, HybridAStarParams):
        values.update(dict(params))
    elif isinstance(params, (JitParams, RitParams, RrtParams)):
        raise TypeError("Hybrid A* params must be HybridAStarParams or a mapping")
    elif isinstance(params, Mapping):
        values.update(params)
    elif params is not None:
        raise TypeError("Hybrid A* params must be HybridAStarParams or a mapping")
    invalid = set(values) - _PARAM_KEYS
    if invalid:
        raise KeyError(f"Unsupported Hybrid A* params: {', '.join(sorted(invalid))}")
    return HybridAStarParams(**cast(Any, values)).validate()


def plan_hybrid_astar(
    problem: ContinuousProblem[State],
    *,
    params: HybridAStarParams
    | JitParams
    | RitParams
    | RrtParams
    | Mapping[str, object]
    | None = None,
    rng: RNG | None = None,
    trace: TraceOptions | None = None,
) -> KinematicPlanResult:
    """Plan a footprint-valid, curvature-bounded path in continuous SE(2)."""
    del rng
    if problem.objective is not None:
        raise ValueError("Hybrid A* uses its native length and gear-change cost")
    if not isinstance(problem.space, KinematicSpace):
        raise TypeError("Hybrid A* requires an SE(2) KinematicSpace")
    resolved_params = _coerce_params(problem, params)
    goal = exact_goal_state(problem.goal, dim=3)
    return run_hybrid_astar(
        problem.space,
        problem.start,
        goal,
        resolved_params,
        trace=trace,
    )


__all__ = ["plan_hybrid_astar"]
