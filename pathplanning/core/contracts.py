"""Core contracts used by planners and reusable planning APIs."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Protocol, TypeAlias

from pathplanning.core.types import Float, Vec

State: TypeAlias = Vec


class ConfigurationSpace(Protocol):
    """Abstract geometric interface expected by sampling-based planners."""

    @property
    def bounds(self) -> tuple[Vec, Vec]:
        """Configuration-space bounds as ``(lower, upper)`` vectors."""
        ...

    @property
    def dim(self) -> int:
        """Dimensionality of the configuration space."""
        ...

    def sample_free(self) -> Vec:
        """Sample one collision-free state from the space."""
        ...

    def is_free(self, state: Vec) -> bool:
        """Return ``True`` when a state is collision free."""
        ...

    def segment_free(self, start: Vec, end: Vec, collision_step: Float) -> bool:
        """Return ``True`` when the line segment is collision free."""
        ...

    def distance(self, start: Vec, end: Vec) -> Float:
        """Distance metric used by the planner."""
        ...

    def steer(self, start: Vec, target: Vec, step_size: Float) -> Vec:
        """Steer from ``start`` toward ``target`` with a bounded step."""
        ...

    def is_goal(self, state: Vec) -> bool:
        """Return ``True`` when a state satisfies the goal condition."""
        ...


def _default_path() -> list[Vec]:
    """Provide a typed default path container for ``PlanResult``."""
    return []


def _default_stats() -> dict[str, object]:
    """Provide a typed default stats container for ``PlanResult``."""
    return {}


@dataclass(slots=True)
class PlanResult:
    """Planner output in a stable, reusable shape."""

    success: bool
    path: list[Vec] = field(default_factory=_default_path)
    iters: int = 0
    nodes: int = 0
    stats: dict[str, object] = field(default_factory=_default_stats)
