"""Small, renderer-independent descriptions of planning scenes."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass, field
from typing import Any, TypeAlias

import numpy as np
from numpy.typing import NDArray


@dataclass(frozen=True, slots=True)
class Rectangle:
    x: float
    y: float
    width: float
    height: float


@dataclass(frozen=True, slots=True)
class Circle:
    x: float
    y: float
    radius: float


@dataclass(frozen=True, slots=True)
class Box:
    lower: tuple[float, float, float]
    upper: tuple[float, float, float]
    orientation: NDArray[np.float64] | None = None


@dataclass(frozen=True, slots=True)
class Sphere:
    center: tuple[float, float, float]
    radius: float


Obstacle: TypeAlias = Rectangle | Circle | Box | Sphere


@dataclass(slots=True)
class Scene:
    """Geometry supplied to a renderer after planning has finished.

    ``bounds`` has shape ``(2, dimension)``. ``occupancy`` is a 2D
    ``(height, width)`` boolean grid; its first cell starts at the lower bound.
    Custom spaces can construct this class directly without exposing their
    internal collision representation to the planner.
    """

    bounds: NDArray[np.float64]
    start: NDArray[np.float64] | None = None
    goal: NDArray[np.float64] | None = None
    obstacles: tuple[Obstacle, ...] = ()
    occupancy: NDArray[np.bool_] | None = field(default=None, repr=False)
    title: str = "Path planning"

    def __post_init__(self) -> None:
        self.bounds = np.array(self.bounds, dtype=float, copy=True)
        if self.bounds.shape not in ((2, 2), (2, 3)):
            raise ValueError("scene bounds must have shape (2, 2) or (2, 3)")
        if not np.all(np.isfinite(self.bounds)) or np.any(self.bounds[0] >= self.bounds[1]):
            raise ValueError("scene bounds must be finite and increasing")
        for name in ("start", "goal"):
            value = getattr(self, name)
            if value is not None:
                point = np.array(value, dtype=float, copy=True)
                if point.shape != (self.dimension,) or not np.all(np.isfinite(point)):
                    raise ValueError(f"scene {name} must be a finite {self.dimension}D point")
                setattr(self, name, point)
        self.obstacles = tuple(
            Box(item.lower, item.upper, np.array(item.orientation, copy=True))
            if isinstance(item, Box) and item.orientation is not None
            else item
            for item in self.obstacles
        )
        if self.occupancy is not None:
            if self.dimension != 2:
                raise ValueError("occupancy is supported only in 2D scenes")
            self.occupancy = np.array(self.occupancy, dtype=np.bool_, copy=True)
            if self.occupancy.ndim != 2:
                raise ValueError("occupancy must be a 2D boolean array")

    @property
    def dimension(self) -> int:
        return self.bounds.shape[1]


def _point(value: Any, dimension: int) -> NDArray[np.float64] | None:
    if value is None or hasattr(value, "is_goal"):
        return None
    if hasattr(value, "state"):
        value = value.state
    try:
        point = np.asarray(value, dtype=float)
    except (TypeError, ValueError):
        return None
    return point if point.shape == (dimension,) else None


def _triple(value: Sequence[float]) -> tuple[float, float, float]:
    return float(value[0]), float(value[1]), float(value[2])


def _grid_scene(graph: Any, start: Any, goal: Any) -> Scene:
    from pathplanning.native.graph import NativeGraph, _GridNodeLabels
    from pathplanning.spaces.grid2d import Grid2DSearchSpace
    from pathplanning.spaces.grid3d import Grid3DSearchSpace

    if type(graph) is Grid2DSearchSpace:
        if graph._is_blocked_callback is not None:
            raise ValueError("grid with a blocked-cell callback needs an explicit Scene")
        cells = graph._blocked_cells
        if graph._occupancy is None:
            obstacles: tuple[Obstacle, ...] = tuple(
                Rectangle(float(x) - 0.5, float(y) - 0.5, 1.0, 1.0)
                for x, y in cells
                if 0 <= x < graph.x_range and 0 <= y < graph.y_range
            )
            occupancy = None
        else:
            obstacles = ()
            occupancy = graph._occupancy
        scene = Scene(
            bounds=np.array(
                [[-0.5, -0.5], [graph.x_range - 0.5, graph.y_range - 0.5]], dtype=float
            ),
            start=_point(start, 2),
            goal=_point(goal, 2),
            obstacles=obstacles,
            occupancy=occupancy,
        )
        if scene.occupancy is not None:
            for x, y in cells:
                if 0 <= x < graph.x_range and 0 <= y < graph.y_range:
                    scene.occupancy[y, x] = True
        return scene
    if type(graph) is Grid3DSearchSpace:
        obstacles = tuple(
            Box(
                (float(x) - 0.5, float(y) - 0.5, float(z) - 0.5),
                (float(x) + 0.5, float(y) + 0.5, float(z) + 0.5),
            )
            for x, y, z in graph.obs
            if 0 <= x < graph.x_range and 0 <= y < graph.y_range and 0 <= z < graph.z_range
        )
        return Scene(
            bounds=np.array(
                [
                    [-0.5, -0.5, -0.5],
                    [graph.x_range - 0.5, graph.y_range - 0.5, graph.z_range - 0.5],
                ],
                dtype=float,
            ),
            start=_point(start, 3),
            goal=_point(goal, 3),
            obstacles=obstacles,
        )
    if type(graph) is NativeGraph and isinstance(graph.node_labels, _GridNodeLabels):
        labels = graph.node_labels
        shape = (labels.width, labels.height, labels.depth)[: labels.dimensions]
        return Scene(
            bounds=np.array(
                [[-0.5] * labels.dimensions, [size - 0.5 for size in shape]], dtype=float
            ),
            start=_point(start, labels.dimensions),
            goal=_point(goal, labels.dimensions),
        )
    raise TypeError("scene_from_problem supports built-in grids; supply Scene for custom graphs")


def scene_from_problem(problem: Any) -> Scene:
    """Snapshot a built-in problem's visible geometry after planning.

    This function only reads built-in geometry. It never walks graph edges or
    calls a custom graph's neighbors/collision predicates.
    """
    from pathplanning.core.contracts import ContinuousProblem, DiscreteProblem
    from pathplanning.spaces.continuous_3d import ContinuousSpace3D
    from pathplanning.spaces.grid2d import Grid2DSamplingSpace

    if isinstance(problem, DiscreteProblem):
        return _grid_scene(problem.graph, problem.start, problem.goal)
    if not isinstance(problem, ContinuousProblem):
        raise TypeError("expected DiscreteProblem or ContinuousProblem")

    space = problem.space
    if type(space) is Grid2DSamplingSpace:
        obstacles: list[Obstacle] = [
            Rectangle(*map(float, item)) for item in (*space.obs_boundary, *space.obs_rectangle)
        ]
        obstacles.extend(Circle(*map(float, item)) for item in space.obs_circle)
        return Scene(
            bounds=np.array([space.x_range, space.y_range], dtype=float).T,
            start=_point(problem.start, 2),
            goal=_point(problem.goal, 2),
            obstacles=tuple(obstacles),
        )
    if type(space) is ContinuousSpace3D:
        obstacles = [
            Box(_triple(item.min_corner), _triple(item.max_corner)) for item in space.aabbs
        ]
        obstacles.extend(Sphere(_triple(item.center), float(item.radius)) for item in space.spheres)
        obstacles.extend(
            Box(
                _triple(item.center - item.extents),
                _triple(item.center + item.extents),
                np.asarray(item.orientation, dtype=float),
            )
            for item in space.obbs
        )
        return Scene(
            bounds=space.bounds,
            start=_point(problem.start, 3),
            goal=_point(problem.goal, 3),
            obstacles=tuple(obstacles),
        )
    raise TypeError("scene_from_problem supports built-in spaces; supply Scene for custom spaces")


__all__ = ["Box", "Circle", "Obstacle", "Rectangle", "Scene", "Sphere", "scene_from_problem"]
