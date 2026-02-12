"""Reusable continuous 3D configuration space and obstacle primitives."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass, field

import numpy as np

from pathplanning.core.contracts import BatchConfigurationSpace
from pathplanning.core.types import BoolArray, Float, FloatArray, Mat, Vec
from pathplanning.spaces.continuous_nd import (
    as_state_nd,
    euclidean_distance_nd,
    in_bounds_nd,
    steer_towards_nd,
)


@dataclass(slots=True)
class AABB:
    """Axis-aligned bounding box in continuous 3D space."""

    min_corner: Vec
    max_corner: Vec

    def __post_init__(self) -> None:
        self.min_corner = as_state_nd(self.min_corner, "min_corner", 3)
        self.max_corner = as_state_nd(self.max_corner, "max_corner", 3)
        if np.any(self.min_corner >= self.max_corner):
            raise ValueError("min_corner must be strictly smaller than max_corner")

    def contains(self, point: Sequence[Float] | FloatArray) -> bool:
        state = as_state_nd(point, "point", 3)
        return bool(np.all(state >= self.min_corner) and np.all(state <= self.max_corner))


@dataclass(slots=True)
class Sphere:
    """Sphere obstacle in continuous 3D space."""

    center: Vec
    radius: Float

    def __post_init__(self) -> None:
        self.center = as_state_nd(self.center, "center", 3)
        self.radius = float(self.radius)
        if self.radius <= 0.0:
            raise ValueError("radius must be > 0")

    def contains(self, point: Sequence[Float] | FloatArray) -> bool:
        state = as_state_nd(point, "point", 3)
        return bool(np.linalg.norm(state - self.center) <= self.radius)


@dataclass(slots=True)
class OBB:
    """Oriented bounding box with center, extents, and orientation matrix."""

    center: Vec
    extents: Vec
    orientation: Mat

    def __post_init__(self) -> None:
        self.center = as_state_nd(self.center, "center", 3)
        self.extents = as_state_nd(self.extents, "extents", 3)
        if np.any(self.extents <= 0.0):
            raise ValueError("extents must all be > 0")
        self.orientation = np.asarray(self.orientation, dtype=float)
        if self.orientation.shape != (3, 3):
            raise ValueError(f"orientation must be shape (3, 3), got {self.orientation.shape}")

    def contains(self, point: Sequence[Float] | FloatArray) -> bool:
        state = as_state_nd(point, "point", 3)
        local = self.orientation.T @ (state - self.center)
        return bool(np.all(np.abs(local) <= self.extents))


@dataclass(slots=True)
class ContinuousSpace3D(BatchConfigurationSpace):
    """Continuous 3D configuration space with collision-checking primitives."""

    lower_bound: Vec
    upper_bound: Vec
    aabbs: tuple[AABB, ...] = field(default_factory=tuple)
    spheres: tuple[Sphere, ...] = field(default_factory=tuple)
    obbs: tuple[OBB, ...] = field(default_factory=tuple)
    goal: Vec | None = None
    goal_tolerance: Float = 0.0
    collision_step: Float = 0.1
    rng: np.random.Generator = field(default_factory=np.random.default_rng, repr=False)
    max_sample_tries: int = 10_000

    def __post_init__(self) -> None:
        self.lower_bound = as_state_nd(self.lower_bound, "lower_bound", 3)
        self.upper_bound = as_state_nd(self.upper_bound, "upper_bound", 3)
        self.aabbs = tuple(self.aabbs)
        self.spheres = tuple(self.spheres)
        self.obbs = tuple(self.obbs)
        if np.any(self.lower_bound >= self.upper_bound):
            raise ValueError("lower_bound must be strictly smaller than upper_bound")
        self.goal_tolerance = float(self.goal_tolerance)
        if self.goal_tolerance < 0.0:
            raise ValueError("goal_tolerance must be >= 0")
        self.collision_step = float(self.collision_step)
        if self.collision_step <= 0.0:
            raise ValueError("collision_step must be > 0")
        if self.goal is not None:
            self.goal = as_state_nd(self.goal, "goal", 3)
        if self.max_sample_tries <= 0:
            raise ValueError("max_sample_tries must be > 0")

    @property
    def bounds(self) -> FloatArray:
        return np.vstack((self.lower_bound, self.upper_bound)).astype(float, copy=False)

    @property
    def dim(self) -> int:
        return 3

    @property
    def dimension(self) -> int:
        return self.dim

    def in_bounds(self, point: Sequence[Float] | FloatArray) -> bool:
        return in_bounds_nd(point, self.lower_bound, self.upper_bound, dim=self.dim)

    def sample_free(self, rng: np.random.Generator) -> Vec:
        for _ in range(self.max_sample_tries):
            candidate = rng.uniform(self.lower_bound, self.upper_bound)
            if self.is_free(candidate):
                return candidate
        raise RuntimeError(
            f"Failed to sample a free point in {self.max_sample_tries} tries. "
            "Check bounds and obstacle occupancy."
        )

    def is_free(self, x: Vec) -> bool:
        point = as_state_nd(x, "x", self.dim)
        if not self.in_bounds(point):
            return False
        if any(obstacle.contains(point) for obstacle in self.aabbs):
            return False
        if any(obstacle.contains(point) for obstacle in self.spheres):
            return False
        return not any(obstacle.contains(point) for obstacle in self.obbs)

    def is_state_valid(self, x: Vec) -> bool:
        return self.is_free(x)

    def segment_free(self, a: Vec, b: Vec, collision_step: Float | None = None) -> bool:
        start_state = as_state_nd(a, "a", self.dim)
        end_state = as_state_nd(b, "b", self.dim)
        step = self.collision_step if collision_step is None else float(collision_step)
        if step <= 0.0:
            raise ValueError("collision_step must be > 0")

        segment_length = self.distance(start_state, end_state)
        if segment_length == 0.0:
            return self.is_free(start_state)

        num_steps = int(np.ceil(segment_length / step))
        for i in range(num_steps + 1):
            alpha = i / num_steps
            sample = start_state + alpha * (end_state - start_state)
            if not self.is_free(sample):
                return False
        return True

    def is_motion_valid(self, a: Vec, b: Vec) -> bool:
        return self.segment_free(a, b)

    def is_motion_valid_batch(self, edges: list[tuple[Vec, Vec]]) -> list[bool]:
        return [self.is_motion_valid(start, end) for start, end in edges]

    def distance(self, a: Vec, b: Vec) -> Float:
        return euclidean_distance_nd(a, b, dim=self.dim)

    def steer(self, a: Vec, b: Vec, step: Float) -> Vec:
        return steer_towards_nd(a, b, step, dim=self.dim)

    def interpolate(self, a: Vec, b: Vec, t: Float) -> Vec:
        start_state = as_state_nd(a, "a", self.dim)
        end_state = as_state_nd(b, "b", self.dim)
        alpha = float(t)
        if alpha < 0.0 or alpha > 1.0:
            raise ValueError("t must be in [0, 1]")
        return start_state + alpha * (end_state - start_state)

    def is_goal(self, x: Vec) -> bool:
        goal = self.goal
        if goal is None:
            return False
        point = as_state_nd(x, "x", self.dim)
        goal_state = as_state_nd(goal, "goal", self.dim)
        return self.distance(point, goal_state) <= self.goal_tolerance

    def sample_free_batch(self, rng: np.random.Generator, n: int) -> Mat:
        if n < 0:
            raise ValueError("n must be >= 0")
        if n == 0:
            return np.empty((0, self.dim), dtype=float)
        samples = [self.sample_free(rng) for _ in range(n)]
        return np.vstack(samples).astype(float, copy=False)

    def is_free_batch(self, x: Mat) -> BoolArray:
        points = np.asarray(x, dtype=float)
        if points.ndim != 2 or points.shape[1] != self.dim:
            raise ValueError(f"x must be shape (n, {self.dim}), got {points.shape}")
        return np.asarray([self.is_free(point) for point in points], dtype=np.bool_)

    def segment_free_batch(self, a: Mat, b: Mat) -> BoolArray:
        starts = np.asarray(a, dtype=float)
        ends = np.asarray(b, dtype=float)
        if starts.ndim != 2 or starts.shape[1] != self.dim:
            raise ValueError(f"a must be shape (n, {self.dim}), got {starts.shape}")
        if ends.ndim != 2 or ends.shape[1] != self.dim:
            raise ValueError(f"b must be shape (n, {self.dim}), got {ends.shape}")
        if starts.shape[0] != ends.shape[0]:
            raise ValueError("a and b must have the same number of rows")
        return np.asarray(
            [self.segment_free(start, end) for start, end in zip(starts, ends, strict=True)],
            dtype=np.bool_,
        )
