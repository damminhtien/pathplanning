"""State-lattice A* over user-provided SE(2) motion primitives."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import fields
from typing import Any, cast

from pathplanning.core.contracts import ContinuousProblem, State, StateLatticeSpace
from pathplanning.core.params import (
    HybridAStarParams,
    JitParams,
    RitParams,
    RrtParams,
    StateLatticeParams,
)
from pathplanning.core.results import KinematicPlanResult
from pathplanning.core.trace import TraceOptions
from pathplanning.core.types import RNG
from pathplanning.native.state_lattice import run_state_lattice
from pathplanning.planners.sampling._internal.continuous import exact_goal_state

_PARAM_KEYS = {field.name for field in fields(StateLatticeParams)}


def _coerce_params(
    problem: ContinuousProblem[State],
    params: (
        StateLatticeParams
        | HybridAStarParams
        | JitParams
        | RitParams
        | RrtParams
        | Mapping[str, object]
        | None
    ),
) -> StateLatticeParams:
    values = dict(problem.params or {})
    if isinstance(params, StateLatticeParams):
        values.update(dict(params))
    elif isinstance(params, (HybridAStarParams, JitParams, RitParams, RrtParams)):
        raise TypeError("State Lattice params must be StateLatticeParams or a mapping")
    elif isinstance(params, Mapping):
        values.update(params)
    elif params is not None:
        raise TypeError("State Lattice params must be StateLatticeParams or a mapping")
    invalid = set(values) - _PARAM_KEYS
    if invalid:
        raise KeyError(f"Unsupported State Lattice params: {', '.join(sorted(invalid))}")
    return StateLatticeParams(**cast(Any, values)).validate()


def plan_state_lattice(
    problem: ContinuousProblem[State],
    *,
    params: (
        StateLatticeParams
        | HybridAStarParams
        | JitParams
        | RitParams
        | RrtParams
        | Mapping[str, object]
        | None
    ) = None,
    rng: RNG | None = None,
    trace: TraceOptions | None = None,
) -> KinematicPlanResult:
    """Find a collision-free path using the robot space's sampled primitives."""
    del rng
    if problem.objective is not None:
        raise ValueError("State Lattice uses its primitive costs and gear-change cost")
    if not isinstance(problem.space, StateLatticeSpace):
        raise TypeError("State Lattice requires a StateLatticeSpace contract")
    resolved_params = _coerce_params(problem, params)
    goal = exact_goal_state(problem.goal, dim=3)
    return run_state_lattice(problem.space, problem.start, goal, resolved_params, trace=trace)


__all__ = ["plan_state_lattice"]
