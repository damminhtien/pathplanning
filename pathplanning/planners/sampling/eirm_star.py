"""Effort-informed multi-query roadmap planning."""

from __future__ import annotations

from collections.abc import Mapping

from pathplanning.core.contracts import ContinuousProblem, State
from pathplanning.core.params import RrtParams
from pathplanning.core.results import PlanResult
from pathplanning.core.trace import TraceOptions
from pathplanning.core.types import RNG
from pathplanning.planners.sampling.prm_star import EirmStarRoadmap, _plan_roadmap


def plan_eirm_star(
    problem: ContinuousProblem[State],
    *,
    params: RrtParams | Mapping[str, object] | None = None,
    rng: RNG | None = None,
    trace: TraceOptions | None = None,
) -> PlanResult:
    """Build and query a reusable native EIRM* roadmap for path length."""
    return _plan_roadmap(
        problem,
        params=params,
        rng=rng,
        trace=trace,
        roadmap_type=EirmStarRoadmap,
        planner_label="EIRM*",
    )


__all__ = ["EirmStarRoadmap", "plan_eirm_star"]
