"""Explicit Estimation Conflict-Based Search for unit-time MAPF."""

from __future__ import annotations

from collections.abc import Mapping
from typing import TypeVar

from pathplanning.core.contracts import MultiAgentProblem
from pathplanning.core.results import MultiAgentPlanResult
from pathplanning.core.trace import TraceOptions
from pathplanning.core.types import RNG
from pathplanning.planners.multi_agent._common import _run_eecbs

N = TypeVar("N")


def plan_eecbs(
    problem: MultiAgentProblem[N],
    *,
    params: Mapping[str, object] | None = None,
    rng: RNG | None = None,
    trace: TraceOptions | None = None,
) -> MultiAgentPlanResult[N]:
    """Plan collision-free agent paths using high-level EES and low-level focal search."""
    return _run_eecbs(problem, params=params, rng=rng, trace=trace)


__all__ = ["plan_eecbs"]
