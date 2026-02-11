"""Reusable continuous 3D configuration space and obstacle primitives."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass, field

import numpy as np
from numpy.typing import NDArray

from pathplanning.core.contracts import ConfigurationSpace, State


def _as_state(value: Sequence[float] | NDArray[np.float64], name: str) -> State:
    state = np.asarray(value, dtype=float)
    if state.shape != (3,):
        raise ValueError(f"{name} must be shape (3,), got {state.shape}")
    return state


@dataclass(slots=True)
class AABB:
    """Axis-aligned bounding box in continuous 3D space."""

    min_corner: Sequence[float] | NDArray[np.float64]
    max_corner: Sequence[float] | NDArray[np.float64]

    def __post_init__(self) -> None:
        self.min_corner = _as_state(self.min_corner, "min_corner")
        self.max_corner = _as_state(self.max_corner, "max_corner")
        if np.any(self.min_corner >= self.max_corner):
            raise ValueError("min_corner must be strictly smaller than max_corner")

    def contains(self, point: Sequence[float] | NDArray[np.float64]) -> bool:
        state = _as_state(point, "point")
        return bool(np.all(state >= self.min_corner) and np.all(state <= self.max_corner))


@dataclass(slots=True)
class Sphere:
    """Sphere obstacle in continuous 3D space."""

    center: Sequence[float] | NDArray[np.float64]
    radius: float

    def __post_init__(self) -> None:
        self.center = _as_state(self.center, "center")
        self.radius = float(self.radius)
        if self.radius <= 0.0:
            raise ValueError("radius must be > 0")

    def contains(self, point: Sequence[float] | NDArray[np.float64]) -> bool:
        state = _as_state(point, "point")
        return bool(np.linalg.norm(state - self.center) <= self.radius)


@dataclass(slots=True)
class OBB:
    """Oriented bounding box with center, extents, and orientation matrix."""

    center: Sequence[float] | NDArray[np.float64]
    extents: Sequence[float] | NDArray[np.float64]
    orientation: NDArray[np.float64]

    def __post_init__(self) -> None:
        self.center = _as_state(self.center, "center")
        self.extents = _as_state(self.extents, "extents")
        if np.any(self.extents <= 0.0):
            raise ValueError("extents must all be > 0")
        self.orientation = np.asarray(self.orientation, dtype=float)
        if self.orientation.shape != (3, 3):
            raise ValueError(f"orientation must be shape (3, 3), got {self.orientation.shape}")

    def contains(self, point: Sequence[float] | NDArray[np.float64]) -> bool:
        state = _as_state(point, "point")
        local = self.orientation.T @ (state - self.center)
        return bool(np.all(np.abs(local) <= self.extents))


@dataclass(slots=True)
class ContinuousSpace3D(ConfigurationSpace):
    """Continuous 3D configuration space with collision-checking primitives."""

    lower_bound: Sequence[float] | NDArray[np.float64]
    upper_bound: Sequence[float] | NDArray[np.float64]
    aabbs: tuple[AABB, ...] = field(default_factory=tuple)
    spheres: tuple[Sphere, ...] = field(default_factory=tuple)
    obbs: tuple[OBB, ...] = field(default_factory=tuple)
    goal: Sequence[float] | NDArray[np.float64] | None = None
    goal_tolerance: float = 0.0
    rng: np.random.Generator = field(default_factory=np.random.default_rng, repr=False)
    max_sample_tries: int = 10_000

    def __post_init__(self) -> None:
        self.lower_bound = _as_state(self.lower_bound, "lower_bound")
        self.upper_bound = _as_state(self.upper_bound, "upper_bound")
        if np.any(self.lower_bound >= self.upper_bound):
            raise ValueError("lower_bound must be strictly smaller than upper_bound")
        self.goal_tolerance = float(self.goal_tolerance)
        if self.goal_tolerance < 0.0:
            raise ValueError("goal_tolerance must be >= 0")
        if self.goal is not None:
            self.goal = _as_state(self.goal, "goal")
        if self.max_sample_tries <= 0:
            raise ValueError("max_sample_tries must be > 0")

    @property
    def bounds(self) -> tuple[State, State]:
        return np.array(self.lower_bound, dtype=float), np.array(self.upper_bound, dtype=float)

    @property
    def dim(self) -> int:
        return 3

    def in_bounds(self, point: Sequence[float] | NDArray[np.float64]) -> bool:
        state = _as_state(point, "point")
        return bool(np.all(state >= self.lower_bound) and np.all(state <= self.upper_bound))

    def sample_free(self) -> State:
        for _ in range(self.max_sample_tries):
            candidate = self.rng.uniform(self.lower_bound, self.upper_bound)
            if self.is_free(candidate):
                return candidate
        raise RuntimeError(
            f"Failed to sample a free point in {self.max_sample_tries} tries. "
            "Check bounds and obstacle occupancy."
        )

    def is_free(self, state: State) -> bool:
        point = _as_state(state, "state")
        if not self.in_bounds(point):
            return False
        if any(obstacle.contains(point) for obstacle in self.aabbs):
            return False
        if any(obstacle.contains(point) for obstacle in self.spheres):
            return False
        return not any(obstacle.contains(point) for obstacle in self.obbs)

    def segment_free(self, start: State, end: State, collision_step: float) -> bool:
        start_state = _as_state(start, "start")
        end_state = _as_state(end, "end")
        step = float(collision_step)
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

    def distance(self, start: State, end: State) -> float:
        start_state = _as_state(start, "start")
        end_state = _as_state(end, "end")
        return float(np.linalg.norm(end_state - start_state))

    def steer(self, start: State, target: State, step_size: float) -> State:
        start_state = _as_state(start, "start")
        target_state = _as_state(target, "target")
        step = float(step_size)
        if step <= 0.0:
            raise ValueError("step_size must be > 0")

        direction = target_state - start_state
        distance = float(np.linalg.norm(direction))
        if distance <= step:
            return target_state
        return start_state + (direction / distance) * step

    def is_goal(self, state: State) -> bool:
        goal = self.goal
        if goal is None:
            return False
        point = _as_state(state, "state")
        goal_state = _as_state(goal, "goal")
        return self.distance(point, goal_state) <= self.goal_tolerance
