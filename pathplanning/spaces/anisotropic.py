"""Reference bounded spaces with constant anisotropic Riemannian costs."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass, field

import numpy as np

from pathplanning.core.types import Float, FloatArray, Vec


@dataclass(slots=True)
class AnisotropicContinuousSpace:
    """N-dimensional box world with a constant symmetric positive metric.

    Obstacles are axis-aligned boxes supplied as ``(lower, upper)`` pairs.
    This small reference model is useful for RIT* examples and tests; collision
    checks and metric calls run through the opt-in Python callback boundary.
    """

    lower_bound: Sequence[Float]
    upper_bound: Sequence[Float]
    metric: Sequence[Sequence[Float]]
    obstacles: tuple[tuple[Sequence[Float], Sequence[Float]], ...] = ()
    collision_step: Float = 0.1
    max_sample_tries: int = 1_000
    _lower: FloatArray = field(init=False, repr=False)
    _upper: FloatArray = field(init=False, repr=False)
    _metric: FloatArray = field(init=False, repr=False)
    _obstacle_bounds: tuple[tuple[FloatArray, FloatArray], ...] = field(init=False, repr=False)
    _eigenvalue_bounds: tuple[float, float] = field(init=False, repr=False)

    def __post_init__(self) -> None:
        self._lower = np.asarray(self.lower_bound, dtype=np.float64)
        self._upper = np.asarray(self.upper_bound, dtype=np.float64)
        self._metric = np.asarray(self.metric, dtype=np.float64)
        dimension = int(self._lower.size)
        if dimension == 0 or self._lower.shape != (dimension,) or self._upper.shape != (dimension,):
            raise ValueError("bounds must be matching non-empty vectors")
        if not np.all(np.isfinite(self._lower)) or not np.all(np.isfinite(self._upper)):
            raise ValueError("bounds must be finite")
        if np.any(self._lower >= self._upper):
            raise ValueError("lower_bound must be strictly smaller than upper_bound")
        if self._metric.shape != (dimension, dimension) or not np.all(np.isfinite(self._metric)):
            raise ValueError("metric must be a finite square matrix matching the bounds")
        if not np.allclose(self._metric, self._metric.T, rtol=1e-10, atol=1e-12):
            raise ValueError("metric must be symmetric")
        eigenvalues = np.linalg.eigvalsh(self._metric)
        if eigenvalues[0] <= 0.0:
            raise ValueError("metric must be positive definite")
        if not np.isfinite(self.collision_step) or self.collision_step <= 0.0:
            raise ValueError("collision_step must be > 0")
        if (
            isinstance(self.max_sample_tries, bool)
            or type(self.max_sample_tries) is not int
            or self.max_sample_tries <= 0
        ):
            raise ValueError("max_sample_tries must be a positive integer")
        self._eigenvalue_bounds = (float(eigenvalues[0]), float(eigenvalues[-1]))
        converted: list[tuple[FloatArray, FloatArray]] = []
        for lower, upper in self.obstacles:
            box_lower = np.asarray(lower, dtype=np.float64)
            box_upper = np.asarray(upper, dtype=np.float64)
            if box_lower.shape != (dimension,) or box_upper.shape != (dimension,):
                raise ValueError("obstacle bounds must match the space dimension")
            if not np.all(np.isfinite(box_lower)) or not np.all(np.isfinite(box_upper)):
                raise ValueError("obstacle bounds must be finite")
            if np.any(box_lower >= box_upper):
                raise ValueError("each obstacle lower bound must be below its upper bound")
            converted.append((box_lower, box_upper))
        self._obstacle_bounds = tuple(converted)

    @property
    def dimension(self) -> int:
        return int(self._lower.size)

    @property
    def distance_metric(self) -> str:
        """The sampling API distance remains Euclidean; RIT* uses the tensor."""
        return "euclidean"

    @property
    def metric_eigenvalue_bounds(self) -> tuple[float, float]:
        """Exact global bounds for this constant metric."""
        return self._eigenvalue_bounds

    @property
    def metric_is_constant(self) -> bool:
        """The reference metric does not depend on configuration."""
        return True

    def metric_tensor(self, _state: Vec) -> FloatArray:
        """Return a copy so native callbacks cannot observe user mutation."""
        return self._metric.copy()

    def sample_free(self, rng: np.random.Generator) -> Vec:
        for _ in range(self.max_sample_tries):
            candidate = rng.uniform(self._lower, self._upper)
            if self.is_state_valid(candidate):
                return candidate
        raise RuntimeError("failed to sample a collision-free state")

    def is_state_valid(self, state: Vec) -> bool:
        point = np.asarray(state, dtype=np.float64)
        if point.shape != (self.dimension,) or not np.all(np.isfinite(point)):
            return False
        if np.any(point < self._lower) or np.any(point > self._upper):
            return False
        return not any(
            np.all(point >= lower) and np.all(point <= upper)
            for lower, upper in self._obstacle_bounds
        )

    def is_motion_valid(self, start: Vec, end: Vec) -> bool:
        return self.is_motion_valid_with_step(start, end, self.collision_step)

    def is_motion_valid_with_step(self, start: Vec, end: Vec, collision_step: Float) -> bool:
        first = np.asarray(start, dtype=np.float64)
        last = np.asarray(end, dtype=np.float64)
        if first.shape != (self.dimension,) or last.shape != (self.dimension,):
            return False
        step = float(collision_step)
        if not np.isfinite(step) or step <= 0.0:
            raise ValueError("collision_step must be > 0")
        length = float(np.linalg.norm(last - first))
        count = max(1, int(np.ceil(length / step)))
        return all(
            self.is_state_valid(first + (last - first) * (index / count))
            for index in range(count + 1)
        )

    def distance(self, start: Vec, end: Vec) -> Float:
        return float(np.linalg.norm(np.asarray(end) - np.asarray(start)))

    def steer(self, start: Vec, target: Vec, step_size: Float) -> Vec:
        first = np.asarray(start, dtype=np.float64)
        last = np.asarray(target, dtype=np.float64)
        delta = last - first
        length = float(np.linalg.norm(delta))
        limit = float(step_size)
        if limit <= 0.0:
            raise ValueError("step_size must be > 0")
        return last.copy() if length <= limit else first + delta * (limit / length)


__all__ = ["AnisotropicContinuousSpace"]
