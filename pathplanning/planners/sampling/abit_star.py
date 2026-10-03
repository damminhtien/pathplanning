"""Thin Python API for native C ABIT*."""

from __future__ import annotations

from collections.abc import Mapping

from pathplanning.core.contracts import ContinuousProblem, State
from pathplanning.core.params import RrtParams
from pathplanning.core.results import PlanResult
from pathplanning.core.trace import TraceOptions
from pathplanning.core.types import RNG
from pathplanning.planners.sampling._internal.bit_engine import (
    BatchInformedTreePlanner,
    IndexFactory,
    run_abit_star,
)


class ABITStar(BatchInformedTreePlanner):
    """Anytime BIT* planner interface backed by the native C core."""

    planner_name = "abit_star"


def plan_abit_star(
    problem: ContinuousProblem[State],
    *,
    params: RrtParams | Mapping[str, object] | None = None,
    rng: RNG | None = None,
    trace: TraceOptions | None = None,
) -> PlanResult:
    """Plan with native ABIT* for an exact goal and additive path length."""
    return run_abit_star(problem, params=params, rng=rng, trace=trace)


__all__ = ["ABITStar", "IndexFactory", "plan_abit_star"]
