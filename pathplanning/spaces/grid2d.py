"""Reusable 2D grid-space building blocks for planners and examples."""

from __future__ import annotations

from collections.abc import Callable, Iterable, Sequence
import math

import numpy as np
from numpy.typing import NDArray

Point2D = tuple[int, int]
Motion2D = tuple[int, int]
BoundaryRect = list[int]
ObstacleRect = list[int]
ObstacleCircle = list[int]
GridCell = Point2D
SampleState2D = tuple[float, float]
BlockedCellPredicate = Callable[[GridCell], bool]

_DEFAULT_8_CONNECTED_MOTIONS: tuple[Motion2D, ...] = (
    (-1, 0),
    (-1, 1),
    (0, 1),
    (1, 1),
    (1, 0),
    (1, -1),
    (0, -1),
    (-1, -1),
)


class Grid2DSearchSpace:
    """Reference typed 2D grid graph used by search-based planners."""

    def __init__(
        self,
        width: int = 51,
        height: int = 31,
        motions: Sequence[Motion2D] | None = None,
        obstacles: Iterable[Point2D] | None = None,
        occupancy: NDArray[np.bool_] | None = None,
        is_blocked: BlockedCellPredicate | None = None,
    ) -> None:
        self.x_range = int(width)
        self.y_range = int(height)
        if self.x_range <= 0 or self.y_range <= 0:
            raise ValueError("width and height must be > 0")

        chosen_motions = _DEFAULT_8_CONNECTED_MOTIONS if motions is None else tuple(motions)
        if not chosen_motions:
            raise ValueError("motions must be non-empty")
        self.motions: tuple[Motion2D, ...] = tuple((int(dx), int(dy)) for dx, dy in chosen_motions)

        self._blocked_cells: set[GridCell] = set(obstacles or [])
        self._occupancy: NDArray[np.bool_] | None = None
        self._is_blocked_callback: BlockedCellPredicate | None = is_blocked

        if occupancy is not None:
            self.set_occupancy(occupancy)

    @property
    def obstacles(self) -> set[GridCell]:
        """Return blocked cells by expanding all configured blockage sources."""
        return self.obs_map()

    def update_obs(self, obs: Iterable[Point2D]) -> None:
        self._blocked_cells = {self._coerce_cell(cell) for cell in obs}

    def set_occupancy(self, occupancy: NDArray[np.bool_]) -> None:
        matrix = np.asarray(occupancy, dtype=np.bool_)
        if matrix.ndim != 2:
            raise ValueError("occupancy must be a 2D boolean matrix")
        if matrix.shape != (self.y_range, self.x_range):
            raise ValueError(
                "occupancy shape must match (height, width); "
                f"expected {(self.y_range, self.x_range)}, got {matrix.shape}"
            )
        self._occupancy = matrix

    def set_blocked_predicate(self, callback: BlockedCellPredicate | None) -> None:
        self._is_blocked_callback = callback

    @staticmethod
    def _coerce_cell(cell: Sequence[int] | GridCell) -> GridCell:
        if len(cell) != 2:
            raise ValueError("cell must have length 2")
        return int(cell[0]), int(cell[1])

    def _in_bounds(self, cell: GridCell) -> bool:
        return 0 <= cell[0] < self.x_range and 0 <= cell[1] < self.y_range

    def _is_blocked(self, cell: GridCell) -> bool:
        if cell in self._blocked_cells:
            return True

        occupancy = self._occupancy
        if occupancy is not None and bool(occupancy[cell[1], cell[0]]):
            return True

        if self._is_blocked_callback is not None:
            return bool(self._is_blocked_callback(cell))
        return False

    def is_valid_node(self, n: GridCell) -> bool:
        cell = self._coerce_cell(n)
        return self._in_bounds(cell) and not self._is_blocked(cell)

    def neighbors(self, n: GridCell) -> Iterable[GridCell]:
        cell = self._coerce_cell(n)
        for dx, dy in self.motions:
            nxt = (cell[0] + dx, cell[1] + dy)
            if self.is_valid_node(nxt):
                yield nxt

    def edge_cost(self, a: GridCell, b: GridCell) -> float:
        src = self._coerce_cell(a)
        dst = self._coerce_cell(b)
        if not self.is_valid_node(src) or not self.is_valid_node(dst):
            return float("inf")
        dx = abs(src[0] - dst[0])
        dy = abs(src[1] - dst[1])
        if dx > 1 or dy > 1 or (dx == 0 and dy == 0):
            return float("inf")
        return math.hypot(float(dx), float(dy))

    def heuristic(self, n: GridCell, goal: GridCell) -> float:
        node = self._coerce_cell(n)
        target = self._coerce_cell(goal)
        return math.hypot(float(target[0] - node[0]), float(target[1] - node[1]))

    def obs_map(self) -> set[Point2D]:
        blocked: set[Point2D] = set()
        for y_coord in range(self.y_range):
            for x_coord in range(self.x_range):
                cell = (x_coord, y_coord)
                if self._is_blocked(cell):
                    blocked.add(cell)
        return blocked


class Grid2DSamplingSpace:
    """Reference 2D continuous space with configurable obstacle primitives."""

    def __init__(
        self,
        x_range: tuple[float, float] = (0.0, 50.0),
        y_range: tuple[float, float] = (0.0, 30.0),
        *,
        obs_boundary: Iterable[BoundaryRect] | None = None,
        obs_circle: Iterable[ObstacleCircle] | None = None,
        obs_rectangle: Iterable[ObstacleRect] | None = None,
        delta: float = 0.5,
        collision_step: float = 0.5,
        max_sample_tries: int = 10_000,
    ) -> None:
        self.x_range = (float(x_range[0]), float(x_range[1]))
        self.y_range = (float(y_range[0]), float(y_range[1]))
        if self.x_range[0] >= self.x_range[1]:
            raise ValueError("x_range min must be < max")
        if self.y_range[0] >= self.y_range[1]:
            raise ValueError("y_range min must be < max")

        self.obs_boundary = [list(item) for item in (obs_boundary or [])]
        self.obs_circle = [list(item) for item in (obs_circle or [])]
        self.obs_rectangle = [list(item) for item in (obs_rectangle or [])]

        self.delta = float(delta)
        if self.delta < 0.0:
            raise ValueError("delta must be >= 0")

        self.collision_step = float(collision_step)
        if self.collision_step <= 0.0:
            raise ValueError("collision_step must be > 0")

        self.max_sample_tries = int(max_sample_tries)
        if self.max_sample_tries <= 0:
            raise ValueError("max_sample_tries must be > 0")

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
            if (
                0 <= px - (ox - delta) <= width + 2 * delta
                and 0 <= py - (oy - delta) <= height + 2 * delta
            ):
                return True

        for ox, oy, width, height in self.obs_boundary:
            if (
                0 <= px - (ox - delta) <= width + 2 * delta
                and 0 <= py - (oy - delta) <= height + 2 * delta
            ):
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
        in_bounds = (
            self.x_range[0] <= px <= self.x_range[1] and self.y_range[0] <= py <= self.y_range[1]
        )
        return in_bounds and not self._inside_obstacle(state)

    def is_motion_valid(self, a: Sequence[float], b: Sequence[float]) -> bool:
        start = self._as_state(a)
        end = self._as_state(b)
        if not self.is_state_valid(start) or not self.is_state_valid(end):
            return False

        segment_length = self.distance(start, end)
        if segment_length == 0.0:
            return self.is_state_valid(start)

        num_steps = int(math.ceil(segment_length / self.collision_step))
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


Grid2DGraph = Grid2DSearchSpace


__all__ = [
    "GridCell",
    "Grid2DGraph",
    "Grid2DSearchSpace",
    "Grid2DSamplingSpace",
    "BlockedCellPredicate",
]
