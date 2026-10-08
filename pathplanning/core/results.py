"""Typed planner result models."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field
from enum import Enum
from typing import Generic, TypeVar

from pathplanning.core.trace import PlannerTrace
from pathplanning.core.types import Mat

N = TypeVar("N")


class StopReason(str, Enum):
    """Normalized planner stop reasons."""

    SUCCESS = "success"
    TIME_BUDGET = "time_budget"
    MAX_ITERS = "max_iters"
    NO_PROGRESS = "no_progress"


def _default_stats() -> dict[str, float]:
    """Provide a typed default stats container for ``PlanResult``."""
    return {}


@dataclass(slots=True)
class PlanResult:
    """Planner output in a stable, reusable shape."""

    success: bool
    path: Mat | None
    best_path: Mat | None
    stop_reason: StopReason
    iters: int
    nodes: int
    stats: Mapping[str, float] = field(default_factory=_default_stats)
    trace: PlannerTrace | None = None


@dataclass(slots=True)
class TemporalPlanResult(PlanResult, Generic[N]):
    """Plan result with graph states and their absolute arrival times."""

    states: tuple[N, ...] = ()
    times: tuple[float, ...] = ()
