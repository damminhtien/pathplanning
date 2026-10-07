"""Lazy any-angle Theta* search on 8-connected grid maps."""

from __future__ import annotations

from collections.abc import Mapping

from pathplanning.core.contracts import DiscreteProblem
from pathplanning.core.results import PlanResult
from pathplanning.core.trace import TraceOptions
from pathplanning.core.types import RNG
from pathplanning.planners.search._grid_utils import Cell
from pathplanning.planners.search.theta_star import _plan_any_angle


def plan_lazy_theta_star(
    problem: DiscreteProblem[Cell],
    *,
    params: Mapping[str, object] | None = None,
    rng: RNG | None = None,
    trace: TraceOptions | None = None,
) -> PlanResult:
    """Run native Lazy Theta* with deferred line-of-sight validation."""
    return _plan_any_angle(
        problem,
        "lazy_theta_star",
        "Lazy Theta*",
        params=params,
        rng=rng,
        trace=trace,
    )


__all__ = ["plan_lazy_theta_star"]
