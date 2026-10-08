"""LaCAM* search for unit-time multi-agent path finding."""

from __future__ import annotations

from collections.abc import Mapping
from typing import TypeVar

from pathplanning.core.contracts import MultiAgentProblem
from pathplanning.core.results import MultiAgentPlanResult
from pathplanning.core.trace import TraceOptions
from pathplanning.core.types import RNG
from pathplanning.planners.multi_agent._common import _run_lacam_star

N = TypeVar("N")


def plan_lacam_star(
    problem: MultiAgentProblem[N],
    *,
    params: Mapping[str, object] | None = None,
    rng: RNG | None = None,
    trace: TraceOptions | None = None,
) -> MultiAgentPlanResult[N]:
    """Find and improve collision-free MAPF paths with lazy configuration search."""
    return _run_lacam_star(problem, params=params, rng=rng, trace=trace)


__all__ = ["plan_lacam_star"]
