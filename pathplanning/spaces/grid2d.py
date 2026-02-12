"""Canonical 2D grid-space definitions shared across planners."""

from __future__ import annotations

from collections.abc import Iterable, Sequence
import math

import numpy as np

Point2D = tuple[int, int]
Motion2D = tuple[int, int]
BoundaryRect = list[int]
ObstacleRect = list[int]
ObstacleCircle = list[int]
GridCell = Point2D
SampleState2D = tuple[float, float]

_DEFAULT_8_CONNECTED_MOTIONS: list[Motion2D] = [
    (-1, 0),
    (-1, 1),
    (0, 1),
    (1, 1),
    (1, 0),
    (1, -1),
    (0, -1),
    (-1, -1),
]


def _default_search_obstacles(width: int, height: int) -> set[Point2D]:
    """Build the default obstacle set used by legacy 2D search planners."""
    obs: set[Point2D] = set()

    for x_coord in range(width):
        obs.add((x_coord, 0))
        obs.add((x_coord, height - 1))

    for y_coord in range(height):
        obs.add((0, y_coord))
        obs.add((width - 1, y_coord))

    for x_coord in range(10, 21):
        obs.add((x_coord, 15))
    for y_coord in range(15):
        obs.add((20, y_coord))

    for y_coord in range(15, 30):
        obs.add((30, y_coord))
    for y_coord in range(16):
        obs.add((40, y_coord))

    return obs


def _default_sampling_obstacle_boundary() -> list[BoundaryRect]:
    return [
        [0, 0, 1, 30],
        [0, 30, 50, 1],
        [1, 0, 50, 1],
        [50, 1, 1, 30],
    ]


def _default_sampling_obstacle_rectangles() -> list[ObstacleRect]:
    return [
        [14, 12, 8, 2],
        [18, 22, 8, 3],
        [26, 7, 2, 12],
        [32, 14, 10, 2],
    ]


def _default_sampling_obstacle_circles() -> list[ObstacleCircle]:
    return [
        [7, 12, 3],
        [46, 20, 2],
        [15, 5, 2],
        [37, 7, 3],
        [37, 23, 3],
    ]


class Grid2DSearchSpace:
    """Reference typed 2D grid graph used by search-based planners."""

    def __init__(
        self,
        width: int = 51,
        height: int = 31,
        motions: Sequence[Motion2D] | None = None,
        obstacles: Iterable[Point2D] | None = None,
    ) -> None:
        self.x_range = int(width)
        self.y_range = int(height)
        self.motions: list[Motion2D] = list(
            _DEFAULT_8_CONNECTED_MOTIONS if motions is None else motions
        )
        self.obs: set[Point2D] = self.obs_map() if obstacles is None else set(obstacles)

    @property
    def obstacles(self) -> set[GridCell]:
        """Return blocked grid cells."""
        return self.obs

    def update_obs(self, obs: Iterable[Point2D]) -> None:
        self.obs = set(obs)

    def is_valid_node(self, n: GridCell) -> bool:
        x_coord, y_coord = n
        in_bounds = 0 <= x_coord < self.x_range and 0 <= y_coord < self.y_range
        return in_bounds and n not in self.obs

    def neighbors(self, n: GridCell) -> Iterable[GridCell]:
        return (
            (n[0] + dx, n[1] + dy)
            for dx, dy in self.motions
            if self.is_valid_node((n[0] + dx, n[1] + dy))
        )

    def edge_cost(self, a: GridCell, b: GridCell) -> float:
        if not self.is_valid_node(a) or not self.is_valid_node(b):
            return float("inf")
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])
        if dx > 1 or dy > 1 or (dx == 0 and dy == 0):
            return float("inf")
        return math.hypot(float(dx), float(dy))

    def heuristic(self, n: GridCell, goal: GridCell) -> float:
        _ = self
        return math.hypot(float(goal[0] - n[0]), float(goal[1] - n[1]))

    def obs_map(self) -> set[Point2D]:
        return _default_search_obstacles(self.x_range, self.y_range)


class Grid2DSamplingSpace:
    """Map model used by sampling-based 2D planners."""

    def __init__(self) -> None:
        self.x_range = (0, 50)
        self.y_range = (0, 30)
        self.obs_boundary = self.default_obs_boundary()
        self.obs_circle = self.default_obs_circle()
        self.obs_rectangle = self.default_obs_rectangle()
        self.delta = 0.5
        self.collision_step = 0.5
        self.max_sample_tries = 10_000

    @staticmethod
    def default_obs_boundary() -> list[BoundaryRect]:
        return _default_sampling_obstacle_boundary()

    @staticmethod
    def default_obs_rectangle() -> list[ObstacleRect]:
        return _default_sampling_obstacle_rectangles()

    @staticmethod
    def default_obs_circle() -> list[ObstacleCircle]:
        return _default_sampling_obstacle_circles()

    @staticmethod
    def _as_state(x: Sequence[float]) -> SampleState2D:
        if len(x) != 2:
            raise ValueError("state must have length 2")
        return float(x[0]), float(x[1])

    def _inside_obstacle(self, x: SampleState2D) -> bool:
        px, py = x
        delta = self.delta

        for cx, cy, radius in self.obs_circle:
            if math.hypot(px - cx, py - cy) <= radius + delta:
                return True

        for ox, oy, width, height in self.obs_rectangle:
            if 0 <= px - (ox - delta) <= width + 2 * delta and 0 <= py - (oy - delta) <= height + 2 * delta:
                return True

        for ox, oy, width, height in self.obs_boundary:
            if 0 <= px - (ox - delta) <= width + 2 * delta and 0 <= py - (oy - delta) <= height + 2 * delta:
                return True

        return False

    def sample_free(self, rng: np.random.Generator) -> SampleState2D:
        x_min, x_max = float(self.x_range[0]), float(self.x_range[1])
        y_min, y_max = float(self.y_range[0]), float(self.y_range[1])
        for _ in range(self.max_sample_tries):
            candidate = (float(rng.uniform(x_min, x_max)), float(rng.uniform(y_min, y_max)))
            if self.is_state_valid(candidate):
                return candidate
        raise RuntimeError(
            f"Failed to sample a valid 2D state in {self.max_sample_tries} tries. "
            "Check bounds and obstacle occupancy."
        )

    def is_state_valid(self, x: Sequence[float]) -> bool:
        state = self._as_state(x)
        px, py = state
        in_bounds = self.x_range[0] <= px <= self.x_range[1] and self.y_range[0] <= py <= self.y_range[1]
        return in_bounds and not self._inside_obstacle(state)

    def is_motion_valid(self, a: Sequence[float], b: Sequence[float]) -> bool:
        start = self._as_state(a)
        end = self._as_state(b)
        if not self.is_state_valid(start) or not self.is_state_valid(end):
            return False

        segment_length = self.distance(start, end)
        if segment_length == 0.0:
            return self.is_state_valid(start)

        step = float(self.collision_step)
        if step <= 0.0:
            raise ValueError("collision_step must be > 0")

        num_steps = int(math.ceil(segment_length / step))
        for i in range(num_steps + 1):
            alpha = i / num_steps
            sample = (
                start[0] + alpha * (end[0] - start[0]),
                start[1] + alpha * (end[1] - start[1]),
            )
            if not self.is_state_valid(sample):
                return False
        return True

    def is_motion_valid_batch(self, edges: list[tuple[SampleState2D, SampleState2D]]) -> list[bool]:
        return [self.is_motion_valid(start, end) for start, end in edges]

    def distance(self, a: Sequence[float], b: Sequence[float]) -> float:
        start = self._as_state(a)
        end = self._as_state(b)
        return math.hypot(end[0] - start[0], end[1] - start[1])

    def steer(self, a: Sequence[float], b: Sequence[float], step_size: float) -> SampleState2D:
        if step_size <= 0.0:
            raise ValueError("step_size must be > 0")
        start = self._as_state(a)
        target = self._as_state(b)
        dist = self.distance(start, target)
        if dist <= step_size:
            return target
        ratio = step_size / dist
        return (
            start[0] + (target[0] - start[0]) * ratio,
            start[1] + (target[1] - start[1]) * ratio,
        )


__all__ = [
    "GridCell",
    "Grid2DSearchSpace",
    "Grid2DSamplingSpace",
]
