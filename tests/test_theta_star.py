"""Focused correctness checks for Theta* visibility and costs."""

from __future__ import annotations

import math

import numpy as np
import pytest

from pathplanning import plan_discrete
from pathplanning.core.contracts import DiscreteProblem
from pathplanning.core.results import StopReason
from pathplanning.spaces.grid2d import Grid2DSearchSpace
from pathplanning.spaces.terrain_grid2d import TerrainCostGrid2D


def _segment_cells(
    first: tuple[float, float], second: tuple[float, float], width: int, height: int
) -> list[tuple[int, int]]:
    """Enumerate closed grid cells touched by a segment using slab intersection."""
    touched = []
    dx, dy = second[0] - first[0], second[1] - first[1]
    for y in range(height):
        for x in range(width):
            lower_t, upper_t = 0.0, 1.0
            for origin, delta, lower, upper in (
                (first[0], dx, x - 0.5, x + 0.5),
                (first[1], dy, y - 0.5, y + 0.5),
            ):
                if delta == 0.0:
                    if origin < lower or origin > upper:
                        lower_t, upper_t = 1.0, 0.0
                        break
                    continue
                entry = (lower - origin) / delta
                exit_ = (upper - origin) / delta
                lower_t = max(lower_t, min(entry, exit_))
                upper_t = min(upper_t, max(entry, exit_))
            if lower_t <= upper_t + 1e-12:
                touched.append((x, y))
    return touched


def _assert_waypoints_visible(problem: DiscreteProblem[tuple[int, int]], path: np.ndarray) -> None:
    graph = problem.graph
    for first, second in zip(path, path[1:]):
        cells = _segment_cells(tuple(first), tuple(second), graph.x_range, graph.y_range)
        assert cells
        assert all(graph.is_valid_node(cell) for cell in cells)


def test_theta_star_returns_visible_any_angle_waypoints() -> None:
    open_problem = DiscreteProblem(Grid2DSearchSpace(9, 7), (0, 0), (8, 5))
    direct = plan_discrete(open_problem, planner="theta_star")
    assert direct.success
    assert direct.path is not None
    assert direct.path.tolist() == [[0.0, 0.0], [8.0, 5.0]]
    assert math.isclose(direct.stats["path_cost"], math.hypot(8.0, 5.0))

    graph = Grid2DSearchSpace(9, 9, obstacles={(4, 4)})
    problem = DiscreteProblem(graph, (1, 1), (7, 7))
    result = plan_discrete(problem, planner="theta_star")
    assert result.success
    assert result.path is not None and len(result.path) > 2
    _assert_waypoints_visible(problem, result.path)
    assert result.stats["line_of_sight_checks"] > 0

    corner_graph = Grid2DSearchSpace(3, 3, obstacles={(1, 0)})
    corner_problem = DiscreteProblem(corner_graph, (0, 0), (1, 1))
    corner_result = plan_discrete(corner_problem, planner="theta_star")
    assert corner_result.success
    assert corner_result.path is not None and len(corner_result.path) > 2
    _assert_waypoints_visible(corner_problem, corner_result.path)


def test_theta_star_integrates_weighted_terrain_along_segments() -> None:
    terrain = np.array([[1.0, 1.0, 4.0, 4.0, 1.0], [1.0] * 5, [1.0] * 5])
    occupancy = np.array(
        [[False] * 5, [True] * 5, [True] * 5],
        dtype=bool,
    )
    graph = TerrainCostGrid2D(terrain, occupancy=occupancy)
    result = plan_discrete(DiscreteProblem(graph, (0, 0), (4, 0)), planner="theta_star")

    assert result.success
    assert result.path is not None
    assert result.path.tolist() == [[0.0, 0.0], [4.0, 0.0]]
    assert result.stats["path_cost"] == 10.0


def test_theta_star_reports_unreachable_and_expansion_limits() -> None:
    graph = Grid2DSearchSpace(9, 9, obstacles={(4, y) for y in range(9)})
    unreachable = plan_discrete(DiscreteProblem(graph, (0, 0), (8, 8)), planner="theta_star")
    assert not unreachable.success
    assert unreachable.stop_reason is StopReason.NO_PROGRESS

    blocked_start = plan_discrete(DiscreteProblem(graph, (4, 0), (8, 8)), planner="theta_star")
    assert not blocked_start.success
    assert blocked_start.stop_reason is StopReason.NO_PROGRESS
    assert blocked_start.stats["cells_checked"] > 0

    budgeted = plan_discrete(
        DiscreteProblem(Grid2DSearchSpace(9, 9, obstacles={(4, 4)}), (1, 1), (7, 7)),
        planner="theta_star",
        params={"max_expansions": 1},
    )
    assert not budgeted.success
    assert budgeted.stop_reason is StopReason.MAX_ITERS

    with pytest.raises(ValueError, match="max_expansions"):
        plan_discrete(
            DiscreteProblem(Grid2DSearchSpace(5, 5), (0, 0), (4, 4)),
            planner="theta_star",
            params={"max_expansions": 0},
        )

    class CustomCostGrid(Grid2DSearchSpace):
        def edge_cost(self, first: tuple[int, int], second: tuple[int, int]) -> float:
            return super().edge_cost(first, second) * 2.0

    with pytest.raises(ValueError, match="built-in uniform or terrain grid cost model"):
        plan_discrete(DiscreteProblem(CustomCostGrid(5, 5), (0, 0), (4, 4)), planner="theta_star")
