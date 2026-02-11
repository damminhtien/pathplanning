"""Core contracts used by planners and reusable planning APIs."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Protocol, TypeAlias

from .types import State


class ConfigurationSpace(Protocol):
    """Abstract geometric interface expected by sampling-based planners."""

    @property
    def bounds(self) -> tuple[State, State]:
        """Configuration-space bounds as ``(lower, upper)`` vectors."""
        ...

    @property
    def dim(self) -> int:
        """Dimensionality of the configuration space."""
        ...

    def sample_free(self) -> State:
        """Sample one collision-free state from the space."""
        ...

    def is_free(self, state: State) -> bool:
        """Return ``True`` when a state is collision free."""
        ...

    def segment_free(self, start: State, end: State, collision_step: float) -> bool:
        """Return ``True`` when the line segment is collision free."""
        ...

    def distance(self, start: State, end: State) -> float:
        """Distance metric used by the planner."""
        ...

    def steer(self, start: State, target: State, step_size: float) -> State:
        """Steer from ``start`` toward ``target`` with a bounded step."""
        ...

    def is_goal(self, state: State) -> bool:
        """Return ``True`` when a state satisfies the goal condition."""
        ...


class StopReason(str, Enum):
    """Canonical planner termination reasons."""

    START_IN_GOAL = "start_in_goal"
    GOAL_REACHED = "goal_reached"
    TIME_BUDGET = "time_budget"
    MAX_ITERS = "max_iters"


StatValue: TypeAlias = float | int | bool | str | None
PlanStats: TypeAlias = dict[str, StatValue]


@dataclass(slots=True)
class PlanResult:
    """Planner output in a stable, reusable shape."""

    success: bool
    path: list[State] = field(default_factory=lambda: [])
    iters: int = 0
    nodes: int = 0
    stats: PlanStats = field(default_factory=lambda: {})
