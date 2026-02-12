"""User-defined 3D continuous space example.

This module demonstrates how users can implement the ``ContinuousSpace``
Protocol directly, without depending on internal environment modules.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass, field
import math

import numpy as np
from numpy.typing import NDArray

from pathplanning.core.contracts import ContinuousProblem, GoalState
from pathplanning.core.types import Float, RNG

Vec3 = NDArray[np.float64]


def _as_vec3(value: Sequence[float] | NDArray[np.float64], name: str) -> Vec3:
    state = np.asarray(value, dtype=float)
    if state.shape != (3,):
        raise ValueError(f"{name} must be shape (3,), got {state.shape}")
    return state


@dataclass(frozen=True, slots=True)
class SphereObstacle:
    """Simple sphere obstacle primitive."""

    center: Vec3
    radius: float

    def __post_init__(self) -> None:
        object.__setattr__(self, "center", _as_vec3(self.center, "center"))
        if self.radius <= 0.0:
            raise ValueError("radius must be > 0")

    def contains(self, point: Sequence[float] | NDArray[np.float64]) -> bool:
        p = _as_vec3(point, "point")
        return bool(np.linalg.norm(p - self.center) <= self.radius)


@dataclass(frozen=True, slots=True)
class BoxObstacle:
    """Axis-aligned box obstacle primitive."""

    min_corner: Vec3
    max_corner: Vec3

    def __post_init__(self) -> None:
        object.__setattr__(self, "min_corner", _as_vec3(self.min_corner, "min_corner"))
        object.__setattr__(self, "max_corner", _as_vec3(self.max_corner, "max_corner"))
        if np.any(self.min_corner >= self.max_corner):
            raise ValueError("min_corner must be strictly less than max_corner")

    def contains(self, point: Sequence[float] | NDArray[np.float64]) -> bool:
        p = _as_vec3(point, "point")
        return bool(np.all(p >= self.min_corner) and np.all(p <= self.max_corner))


@dataclass(slots=True)
class CustomContinuousSpace3D:
    """User-owned ``ContinuousSpace`` implementation."""

    lower_bound: Vec3
    upper_bound: Vec3
    spheres: tuple[SphereObstacle, ...] = field(default_factory=tuple)
    boxes: tuple[BoxObstacle, ...] = field(default_factory=tuple)
    collision_step: float = 0.15
    max_sample_tries: int = 10_000

    def __post_init__(self) -> None:
        self.lower_bound = _as_vec3(self.lower_bound, "lower_bound")
        self.upper_bound = _as_vec3(self.upper_bound, "upper_bound")
        self.spheres = tuple(self.spheres)
        self.boxes = tuple(self.boxes)
        if np.any(self.lower_bound >= self.upper_bound):
            raise ValueError("lower_bound must be strictly smaller than upper_bound")
        if self.collision_step <= 0.0:
            raise ValueError("collision_step must be > 0")
        if self.max_sample_tries <= 0:
            raise ValueError("max_sample_tries must be > 0")

    @property
    def dimension(self) -> int:
        return 3

    @property
    def bounds(self) -> NDArray[np.float64]:
        return np.vstack((self.lower_bound, self.upper_bound)).astype(float, copy=False)

    def sample_free(self, rng: RNG) -> Vec3:
        for _ in range(self.max_sample_tries):
            candidate = rng.uniform(self.lower_bound, self.upper_bound)
            if self.is_state_valid(candidate):
                return candidate
        raise RuntimeError(
            f"Unable to sample valid state in {self.max_sample_tries} tries. "
            "Check obstacle occupancy and bounds."
        )

    def is_state_valid(self, x: Vec3) -> bool:
        point = _as_vec3(x, "x")
        in_bounds = bool(np.all(point >= self.lower_bound) and np.all(point <= self.upper_bound))
        if not in_bounds:
            return False
        if any(sphere.contains(point) for sphere in self.spheres):
            return False
        if any(box.contains(point) for box in self.boxes):
            return False
        return True

    def is_motion_valid(self, a: Vec3, b: Vec3) -> bool:
        start = _as_vec3(a, "a")
        end = _as_vec3(b, "b")
        if not self.is_state_valid(start) or not self.is_state_valid(end):
            return False

        segment_length = self.distance(start, end)
        if segment_length == 0.0:
            return self.is_state_valid(start)

        steps = int(math.ceil(segment_length / self.collision_step))
        for i in range(steps + 1):
            t = i / steps
            sample = start + t * (end - start)
            if not self.is_state_valid(sample):
                return False
        return True

    def is_motion_valid_batch(self, edges: list[tuple[Vec3, Vec3]]) -> list[bool]:
        return [self.is_motion_valid(start, end) for start, end in edges]

    def distance(self, a: Vec3, b: Vec3) -> Float:
        start = _as_vec3(a, "a")
        end = _as_vec3(b, "b")
        return float(np.linalg.norm(end - start))

    def steer(self, a: Vec3, b: Vec3, step_size: Float) -> Vec3:
        if step_size <= 0.0:
            raise ValueError("step_size must be > 0")
        start = _as_vec3(a, "a")
        target = _as_vec3(b, "b")
        direction = target - start
        norm = float(np.linalg.norm(direction))
        if norm <= step_size:
            return target
        return start + (direction / norm) * float(step_size)


def build_demo_3d_world() -> tuple[CustomContinuousSpace3D, Vec3, Vec3]:
    """Build one custom world and return ``(space, start, goal)``."""
    space = CustomContinuousSpace3D(
        lower_bound=np.array([0.0, 0.0, 0.0], dtype=float),
        upper_bound=np.array([20.0, 20.0, 6.0], dtype=float),
        spheres=(
            SphereObstacle(center=np.array([6.0, 7.0, 2.5], dtype=float), radius=1.5),
            SphereObstacle(center=np.array([12.0, 15.0, 2.5], dtype=float), radius=2.2),
        ),
        boxes=(
            BoxObstacle(
                min_corner=np.array([8.5, 3.0, 0.0], dtype=float),
                max_corner=np.array([10.5, 12.0, 5.0], dtype=float),
            ),
            BoxObstacle(
                min_corner=np.array([3.5, 13.0, 0.0], dtype=float),
                max_corner=np.array([5.0, 19.5, 5.5], dtype=float),
            ),
        ),
        collision_step=0.12,
    )

    start = np.array([2.0, 2.0, 1.0], dtype=float)
    goal = np.array([18.0, 17.0, 1.0], dtype=float)
    return space, start, goal


def build_demo_problem() -> ContinuousProblem[Vec3]:
    """Build a fully-typed ``ContinuousProblem`` using the custom space."""
    space, start, goal = build_demo_3d_world()
    goal_region = GoalState(state=goal, radius=0.8, distance_fn=space.distance)
    return ContinuousProblem(space=space, start=start, goal=goal_region)


if __name__ == "__main__":
    from pathplanning.api import plan_continuous

    problem = build_demo_problem()
    result = plan_continuous(
        problem,
        planner="rrt_star",
        params={"max_iters": 1500, "step_size": 0.8, "goal_sample_rate": 0.2},
    )

    print(f"success={result.success}")
    print(f"stop_reason={result.stop_reason.value}")
    print(f"nodes={result.nodes}, iters={result.iters}")
    if result.path is not None:
        print(f"path_points={len(result.path)}")
