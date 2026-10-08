"""Safe Interval Path Planning with Interval Projection for kinodynamic graphs."""

from __future__ import annotations

from collections.abc import Mapping
from typing import TypeVar

from pathplanning.core.contracts import TemporalProblem
from pathplanning.core.results import TemporalPlanResult
from pathplanning.core.trace import TraceOptions
from pathplanning.core.types import RNG
from pathplanning.planners.temporal.sipp import _plan_sipp

N = TypeVar("N")


def plan_kinodynamic_sipp(
    problem: TemporalProblem[N],
    *,
    params: Mapping[str, object] | None = None,
    rng: RNG | None = None,
    trace: TraceOptions | None = None,
) -> TemporalPlanResult[N]:
    """Search motion-primitive states while propagating wait intervals."""
    return _plan_sipp(
        problem,
        params=params,
        rng=rng,
        trace=trace,
        kinodynamic=True,
    )


__all__ = ["plan_kinodynamic_sipp"]
