"""Focused correctness checks for Lazy Theta* validation and repair."""

from __future__ import annotations

import math

import numpy as np

from pathplanning import plan_discrete
from pathplanning.core.contracts import DiscreteProblem
from pathplanning.core.results import StopReason
from pathplanning.spaces.grid2d import Grid2DSearchSpace
from pathplanning.spaces.terrain_grid2d import TerrainCostGrid2D


def _assert_waypoints_visible(problem: DiscreteProblem, path: np.ndarray) -> None:
    graph = problem.graph
    for first, second in zip(path, path[1:]):
        dx, dy = second - first
        for y in range(graph.y_range):
            for x in range(graph.x_range):
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
                    assert graph.is_valid_node((x, y))


def test_lazy_theta_star_returns_visible_path_with_fewer_visibility_checks() -> None:
    graph = Grid2DSearchSpace(9, 9, obstacles={(4, 4)})
    problem = DiscreteProblem(graph, (1, 1), (7, 7))

    lazy = plan_discrete(problem, planner="lazy_theta_star")
    theta = plan_discrete(problem, planner="theta_star")

    assert lazy.success
    assert lazy.path is not None and len(lazy.path) > 2
    _assert_waypoints_visible(problem, lazy.path)
    assert lazy.stats["line_of_sight_checks"] == lazy.iters
    assert 0 < lazy.stats["line_of_sight_checks"] < theta.stats["line_of_sight_checks"]

    corner_graph = Grid2DSearchSpace(3, 3, obstacles={(1, 0)})
    corner_problem = DiscreteProblem(corner_graph, (0, 0), (1, 1))
    corner = plan_discrete(corner_problem, planner="lazy_theta_star")
    assert corner.success
    assert corner.path is not None
    _assert_waypoints_visible(corner_problem, corner.path)


def test_lazy_theta_star_integrates_weighted_terrain() -> None:
    terrain = np.array([[1.0, 1.0, 4.0, 4.0, 1.0], [1.0] * 5, [1.0] * 5])
    occupancy = np.array([[False] * 5, [True] * 5, [True] * 5], dtype=bool)
    graph = TerrainCostGrid2D(terrain, occupancy=occupancy)

    result = plan_discrete(
        DiscreteProblem(graph, (0, 0), (4, 0)),
        planner="lazy_theta_star",
    )

    assert result.success
    assert result.path is not None
    assert result.path.tolist() == [[0.0, 0.0], [4.0, 0.0]]
    assert math.isclose(result.stats["path_cost"], 10.0)


def test_lazy_theta_star_handles_unreachable_goals_and_expansion_limits() -> None:
    graph = Grid2DSearchSpace(9, 9, obstacles={(4, y) for y in range(9)})
    unreachable = plan_discrete(
        DiscreteProblem(graph, (0, 0), (8, 8)),
        planner="lazy_theta_star",
    )
    assert not unreachable.success
    assert unreachable.stop_reason is StopReason.NO_PROGRESS

    blocked_start = plan_discrete(
        DiscreteProblem(graph, (4, 0), (8, 8)),
        planner="lazy_theta_star",
    )
    assert not blocked_start.success
    assert blocked_start.stop_reason is StopReason.NO_PROGRESS

    trivial = plan_discrete(
        DiscreteProblem(Grid2DSearchSpace(3, 3), (1, 1), (1, 1)),
        planner="lazy_theta_star",
    )
    assert trivial.success
    assert trivial.path is not None and trivial.path.tolist() == [[1.0, 1.0]]
    assert trivial.stats["path_cost"] == 0.0

    budgeted = plan_discrete(
        DiscreteProblem(Grid2DSearchSpace(9, 9, obstacles={(4, 4)}), (1, 1), (7, 7)),
        planner="lazy_theta_star",
        params={"max_expansions": 1},
    )
    assert not budgeted.success
    assert budgeted.stop_reason is StopReason.MAX_ITERS
