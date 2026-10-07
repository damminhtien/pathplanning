"""Shared helpers for grid-specialized search planners."""

from __future__ import annotations

from collections.abc import Mapping
from typing import TypeAlias

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


def _max_expansions(params: Mapping[str, object] | None) -> int | None:
    value = None if params is None else params.get("max_expansions")
    if value is None:
        return None
    if isinstance(value, bool) or type(value) is not int or value <= 0:
        raise ValueError("max_expansions must be a positive integer")
    return value
