"""BIT*: batch sampling with ordered search over an implicit random graph."""

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
    run_bit_star,
)


class BITStar(BatchInformedTreePlanner):
    """Public BIT* planner class."""


def plan_bit_star(
    problem: ContinuousProblem[State],
    *,
    params: RrtParams | Mapping[str, object] | None = None,
    rng: RNG | None = None,
    trace: TraceOptions | None = None,
) -> PlanResult:
    """Plan with BIT* using batched samples and ordered edge search."""
    return run_bit_star(problem, params=params, rng=rng, trace=trace)


__all__ = ["BITStar", "BatchInformedTreePlanner", "IndexFactory", "plan_bit_star"]
