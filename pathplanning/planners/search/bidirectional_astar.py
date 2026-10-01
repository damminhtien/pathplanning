"""Legacy import path for the bidirectional Dijkstra planner.

This module is kept for source compatibility. The implementation is
bidirectional Dijkstra and does not use a heuristic.
"""

from __future__ import annotations

from collections.abc import Mapping
from typing import TypeVar

from pathplanning.core.contracts import DiscreteProblem
from pathplanning.core.results import PlanResult
from pathplanning.core.types import RNG
from pathplanning.planners.search.bidirectional_dijkstra import plan_bidirectional_dijkstra

N = TypeVar("N")


def plan_bidirectional_astar(
    problem: DiscreteProblem[N],
    *,
    params: Mapping[str, object] | None = None,
    rng: RNG | None = None,
) -> PlanResult:
    """Compatibility alias; runs bidirectional Dijkstra, not bidirectional A*."""
    return plan_bidirectional_dijkstra(problem, params=params, rng=rng)


__all__ = ["plan_bidirectional_astar"]
