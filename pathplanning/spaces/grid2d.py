"""Canonical 2D grid-space definitions shared across planners."""

from __future__ import annotations

from collections.abc import Iterable, Sequence

Point2D = tuple[int, int]
Motion2D = tuple[int, int]
BoundaryRect = list[int]
ObstacleRect = list[int]
ObstacleCircle = list[int]

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
    """Grid-space model used by search-based 2D planners."""

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

    def update_obs(self, obs: Iterable[Point2D]) -> None:
        self.obs = set(obs)

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

    @staticmethod
    def default_obs_boundary() -> list[BoundaryRect]:
        return _default_sampling_obstacle_boundary()

    @staticmethod
    def default_obs_rectangle() -> list[ObstacleRect]:
        return _default_sampling_obstacle_rectangles()

    @staticmethod
    def default_obs_circle() -> list[ObstacleCircle]:
        return _default_sampling_obstacle_circles()


__all__ = [
    "Grid2DSearchSpace",
    "Grid2DSamplingSpace",
]
