"""Shared helpers for grid-specialized search planners."""

from __future__ import annotations

from collections.abc import Mapping
from typing import TypeAlias

import numpy as np
from numpy.typing import NDArray

from pathplanning.core.contracts import DiscreteProblem
from pathplanning.spaces.grid2d import Grid2DSearchSpace

Cell: TypeAlias = tuple[int, int]
_MOVES: tuple[Cell, ...] = (
    (1, 0),
    (0, 1),
    (-1, 0),
    (0, -1),
    (1, 1),
    (-1, 1),
    (-1, -1),
    (1, -1),
)


def _grid(problem: DiscreteProblem[Cell]) -> Grid2DSearchSpace:
    graph = problem.graph
    if not isinstance(graph, Grid2DSearchSpace):
        raise TypeError("planner requires a Grid2DSearchSpace")
    motions = getattr(graph, "motions", None)
    if motions is None or len(motions) != len(_MOVES) or set(motions) != set(_MOVES):
        raise ValueError("planner requires the standard 8-connected grid motions")
    return graph


def _native_valid_nodes(graph: Grid2DSearchSpace) -> NDArray[np.uint8]:
    """Return a flattened C-order validity mask without per-cell Python calls."""
    width, height = graph.x_range, graph.y_range
    callback = graph._is_blocked_callback
    if callback is not None:
        return np.fromiter(
            (graph.is_valid_node((x, y)) for y in range(height) for x in range(width)),
            dtype=np.uint8,
            count=width * height,
        )

    occupancy = graph._occupancy
    if occupancy is None:
        valid = np.ones(width * height, dtype=np.uint8)
    else:
        valid = np.logical_not(occupancy).reshape(-1).astype(np.uint8)

    for x, y in graph._blocked_cells:
        if 0 <= x < width and 0 <= y < height:
            valid[y * width + x] = 0
    return valid


def _max_expansions(params: Mapping[str, object] | None) -> int | None:
    value = None if params is None else params.get("max_expansions")
    if value is None:
        return None
    if isinstance(value, bool) or type(value) is not int or value <= 0:
        raise ValueError("max_expansions must be a positive integer")
    return value
