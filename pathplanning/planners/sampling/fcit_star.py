"""Thin Python API for native Fully Connected Informed Trees."""

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
    run_fcit_star,
)


class FCITStar(BatchInformedTreePlanner):
    """Fully connected informed search backed by the native C core."""

    planner_name = "fcit_star"


def plan_fcit_star(
    problem: ContinuousProblem[State],
    *,
    params: RrtParams | Mapping[str, object] | None = None,
    rng: RNG | None = None,
    trace: TraceOptions | None = None,
) -> PlanResult:
    """Plan with FCIT* over a batch-refined fully connected graph."""
    return run_fcit_star(problem, params=params, rng=rng, trace=trace)


__all__ = ["FCITStar", "IndexFactory", "plan_fcit_star"]
