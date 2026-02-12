"""Core contracts used by planners and reusable planning APIs."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Protocol, TypeAlias, runtime_checkable

import numpy as np

from pathplanning.core.types import BoolArray, Float, FloatArray, Mat, Vec

State: TypeAlias = Vec


class ConfigurationSpace(Protocol):
    """Abstract geometric interface expected by sampling-based planners."""

    @property
    def bounds(self) -> FloatArray:
        """Configuration-space bounds matrix with shape ``(2, dim)``."""
        ...

    @property
    def dim(self) -> int:
        """Dimensionality of the configuration space."""
        ...

    def sample_free(self, rng: np.random.Generator) -> Vec:
        """Sample one collision-free state from the space."""
        ...

    def is_free(self, x: Vec) -> bool:
        """Return ``True`` when a state is collision free."""
        ...

    def segment_free(self, a: Vec, b: Vec) -> bool:
        """Return ``True`` when the line segment is collision free."""
        ...

    def distance(self, a: Vec, b: Vec) -> Float:
        """Distance metric used by the planner."""
        ...

    def steer(self, a: Vec, b: Vec, step: Float) -> Vec:
        """Steer from ``a`` toward ``b`` with a bounded step."""
        ...

    def is_goal(self, x: Vec) -> bool:
        """Return ``True`` when a state satisfies the goal condition."""
        ...


@runtime_checkable
class BatchConfigurationSpace(ConfigurationSpace, Protocol):
    """Optional batch operations that planners can use when available."""

    def sample_free_batch(self, rng: np.random.Generator, n: int) -> Mat:
        """Sample ``n`` collision-free states as a matrix with shape ``(n, dim)``."""
        ...

    def is_free_batch(self, x: Mat) -> BoolArray:
        """Return collision-free flags for each state in ``x``."""
        ...

    def segment_free_batch(self, a: Mat, b: Mat) -> BoolArray:
        """Return segment-validity flags for each start/end pair."""
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
