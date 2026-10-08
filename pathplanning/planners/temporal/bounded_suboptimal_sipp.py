"""Focal-search bounded-suboptimal Safe Interval Path Planning."""

from __future__ import annotations

from collections.abc import Mapping
import math
from typing import TypeVar

import numpy as np

from pathplanning.core.contracts import TemporalProblem
from pathplanning.core.results import TemporalPlanResult
from pathplanning.core.trace import TraceOptions
from pathplanning.core.types import RNG
from pathplanning.planners.temporal.sipp import _plan_sipp

N = TypeVar("N")
_DEFAULT_WEIGHT = 1.5


def _suboptimality_weight(params: Mapping[str, object] | None) -> float:
    value = _DEFAULT_WEIGHT if params is None else params.get("w", _DEFAULT_WEIGHT)
    if isinstance(value, bool) or not isinstance(value, (int, float, np.integer, np.floating)):
        raise TypeError("w must be a finite number greater than or equal to 1")
    weight = float(value)
    if not math.isfinite(weight) or weight < 1.0:
        raise ValueError("w must be a finite number greater than or equal to 1")
    return weight


def plan_bounded_suboptimal_sipp(
    problem: TemporalProblem[N],
    *,
    params: Mapping[str, object] | None = None,
    rng: RNG | None = None,
    trace: TraceOptions | None = None,
) -> TemporalPlanResult[N]:
    """Search FOCAL states by hop count while preserving a w-bound."""
    merged_params = dict(problem.params or {})
    if params is not None:
        merged_params.update(params)
    weight = _suboptimality_weight(merged_params)
    return _plan_sipp(
        problem,
        params=merged_params,
        rng=rng,
        trace=trace,
        suboptimality_weight=weight,
    )


__all__ = ["plan_bounded_suboptimal_sipp"]
