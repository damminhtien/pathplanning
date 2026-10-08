"""Reference planar serial-arm model with Jacobian and self-collision checks."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass, field
import math

import numpy as np

from pathplanning.core.types import Float, FloatArray, Vec


def _cross(first: FloatArray, second: FloatArray) -> float:
    return float(first[0] * second[1] - first[1] * second[0])


def _point_segment_distance(point: FloatArray, start: FloatArray, end: FloatArray) -> float:
    segment = end - start
    length_squared = float(segment @ segment)
    if length_squared == 0.0:
        return float(np.linalg.norm(point - start))
    fraction = float(np.clip((point - start) @ segment / length_squared, 0.0, 1.0))
    return float(np.linalg.norm(point - (start + fraction * segment)))


def _segment_distance(
    first_start: FloatArray,
    first_end: FloatArray,
    second_start: FloatArray,
    second_end: FloatArray,
) -> float:
    first = first_end - first_start
    second = second_end - second_start
    denominator = _cross(first, second)
    if abs(denominator) > 1e-12:
        offset = second_start - first_start
        first_fraction = _cross(offset, second) / denominator
        second_fraction = _cross(offset, first) / denominator
        if 0.0 <= first_fraction <= 1.0 and 0.0 <= second_fraction <= 1.0:
            return 0.0
    return min(
        _point_segment_distance(first_start, second_start, second_end),
        _point_segment_distance(first_end, second_start, second_end),
        _point_segment_distance(second_start, first_start, first_end),
        _point_segment_distance(second_end, first_start, first_end),
    )


@dataclass(slots=True)
class PlanarManipulatorSpace:
    """Bounded planar serial arm with circular obstacles and link thickness.

    Joint coordinates are ordinary bounded real values. The model supplies a
    2-by-N end-effector Jacobian and rejects both workspace and non-adjacent
    link collisions, making it a small reference for JIT* integration.
    """

    joint_lower: Sequence[Float]
    joint_upper: Sequence[Float]
    link_lengths: Sequence[Float]
    workspace_lower: Sequence[Float] = (-3.0, -3.0)
    workspace_upper: Sequence[Float] = (3.0, 3.0)
    obstacles: tuple[tuple[Float, Float, Float], ...] = ()
    link_radius: Float = 0.025
    collision_step: Float = 0.05
    max_sample_tries: int = 1_000
    _lower: FloatArray = field(init=False, repr=False)
    _upper: FloatArray = field(init=False, repr=False)
    _lengths: FloatArray = field(init=False, repr=False)
    _workspace_lower: FloatArray = field(init=False, repr=False)
    _workspace_upper: FloatArray = field(init=False, repr=False)
    _obstacles: tuple[tuple[FloatArray, float], ...] = field(init=False, repr=False)

    def __post_init__(self) -> None:
        self._lower = np.asarray(self.joint_lower, dtype=np.float64)
        self._upper = np.asarray(self.joint_upper, dtype=np.float64)
        self._lengths = np.asarray(self.link_lengths, dtype=np.float64)
        self._workspace_lower = np.asarray(self.workspace_lower, dtype=np.float64)
        self._workspace_upper = np.asarray(self.workspace_upper, dtype=np.float64)
        dimension = int(self._lengths.size)
        if dimension < 3 or self._lower.shape != (dimension,) or self._upper.shape != (dimension,):
            raise ValueError("joint limits and link_lengths must have matching size >= 3")
        if (
            not np.all(np.isfinite(self._lower))
            or not np.all(np.isfinite(self._upper))
            or np.any(self._lower >= self._upper)
        ):
            raise ValueError("joint limits must be finite and strictly ordered")
        if not np.all(np.isfinite(self._lengths)) or np.any(self._lengths <= 0.0):
            raise ValueError("link_lengths must be positive and finite")
        if (
            self._workspace_lower.shape != (2,)
            or self._workspace_upper.shape != (2,)
            or not np.all(np.isfinite(self._workspace_lower))
            or not np.all(np.isfinite(self._workspace_upper))
            or np.any(self._workspace_lower >= self._workspace_upper)
        ):
            raise ValueError("workspace bounds must be ordered finite 2D vectors")
        if not np.all(np.isfinite([self.link_radius, self.collision_step])):
            raise ValueError("link_radius and collision_step must be finite")
        if self.link_radius <= 0.0 or self.collision_step <= 0.0:
            raise ValueError("link_radius and collision_step must be > 0")
        if isinstance(self.max_sample_tries, bool) or type(self.max_sample_tries) is not int:
            raise TypeError("max_sample_tries must be an integer")
        if self.max_sample_tries <= 0:
            raise ValueError("max_sample_tries must be > 0")
        converted: list[tuple[FloatArray, float]] = []
        for x, y, radius in self.obstacles:
            values = np.asarray([x, y, radius], dtype=np.float64)
            if not np.all(np.isfinite(values)) or radius <= 0.0:
                raise ValueError("obstacles require finite centers and positive radii")
            converted.append((values[:2].copy(), float(radius)))
        self._obstacles = tuple(converted)

    @property
    def dimension(self) -> int:
        return int(self._lengths.size)

    @property
    def distance_metric(self) -> str:
        return "euclidean"

    def distance(self, first: Vec, second: Vec) -> float:
        return float(np.linalg.norm(np.asarray(second) - np.asarray(first)))

    def steer(self, first: Vec, target: Vec, step_size: Float) -> FloatArray:
        start = np.asarray(first, dtype=np.float64)
        end = np.asarray(target, dtype=np.float64)
        delta = end - start
        distance = float(np.linalg.norm(delta))
        if distance <= step_size:
            return end.copy()
        return start + delta * (float(step_size) / distance)

    def sample_free(self, rng: np.random.Generator) -> Vec:
        for _ in range(self.max_sample_tries):
            state = rng.uniform(self._lower, self._upper)
            if self.is_state_valid(state):
                return state
        raise RuntimeError("failed to sample a collision-free manipulator state")

    def jacobian(self, state: Vec) -> FloatArray:
        """Return the planar end-effector Jacobian for all revolute joints."""
        angles = np.asarray(state, dtype=np.float64)
        if angles.shape != (self.dimension,) or not np.all(np.isfinite(angles)):
            raise ValueError("state must be a finite joint vector")
        cumulative = np.cumsum(angles)
        jacobian = np.zeros((2, self.dimension), dtype=np.float64)
        for joint in range(self.dimension):
            jacobian[0, joint] = -float(self._lengths[joint:] @ np.sin(cumulative[joint:]))
            jacobian[1, joint] = float(self._lengths[joint:] @ np.cos(cumulative[joint:]))
        return jacobian

    def _links(self, state: FloatArray) -> list[tuple[FloatArray, FloatArray]]:
        points = [np.zeros(2, dtype=np.float64)]
        angle = 0.0
        for joint, length in zip(state, self._lengths, strict=True):
            angle += float(joint)
            points.append(
                points[-1] + float(length) * np.asarray([math.cos(angle), math.sin(angle)])
            )
        return list(zip(points[:-1], points[1:], strict=True))

    def is_state_valid(self, state: Vec) -> bool:
        angles = np.asarray(state, dtype=np.float64)
        if (
            angles.shape != (self.dimension,)
            or not np.all(np.isfinite(angles))
            or np.any(angles < self._lower)
            or np.any(angles > self._upper)
        ):
            return False
        links = self._links(angles)
        for start, end in links:
            if (
                np.any(start < self._workspace_lower)
                or np.any(start > self._workspace_upper)
                or np.any(end < self._workspace_lower)
                or np.any(end > self._workspace_upper)
            ):
                return False
            if any(
                _point_segment_distance(center, start, end) < radius + self.link_radius
                for center, radius in self._obstacles
            ):
                return False
        for first in range(len(links)):
            for second in range(first + 2, len(links)):
                if _segment_distance(*links[first], *links[second]) < 2.0 * self.link_radius:
                    return False
        return True

    def is_motion_valid(self, start: Vec, end: Vec) -> bool:
        return self.is_motion_valid_with_step(start, end, self.collision_step)

    def is_motion_valid_with_step(self, start: Vec, end: Vec, collision_step: Float) -> bool:
        first = np.asarray(start, dtype=np.float64)
        last = np.asarray(end, dtype=np.float64)
        distance = float(np.linalg.norm(last - first))
        steps = max(1, int(math.ceil(distance / float(collision_step))))
        return all(
            self.is_state_valid(first + (last - first) * (index / steps))
            for index in range(steps + 1)
        )


__all__ = ["PlanarManipulatorSpace"]
