"""Core contracts used by planners and reusable planning APIs."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Protocol, TypeAlias

import numpy as np
from numpy.typing import NDArray

State: TypeAlias = NDArray[np.float64]


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


def _default_path() -> list[State]:
    """Provide a typed default path container for ``PlanResult``."""
    return []


def _default_stats() -> dict[str, object]:
    """Provide a typed default stats container for ``PlanResult``."""
    return {}


@dataclass(slots=True)
class PlanResult:
    """Planner output in a stable, reusable shape."""

    success: bool
    path: list[State] = field(default_factory=_default_path)
    iters: int = 0
    nodes: int = 0
    stats: dict[str, object] = field(default_factory=_default_stats)
