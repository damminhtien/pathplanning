"""Correctness checks for Weighted Jump Point Search."""

from __future__ import annotations

import heapq
import math

import numpy as np
import pytest

from pathplanning.core.contracts import DiscreteProblem
from pathplanning.core.results import StopReason
from pathplanning.planners.search.jump_point_weighted import plan_jpsw
from pathplanning.spaces.grid2d import Grid2DSearchSpace
from pathplanning.spaces.terrain_grid2d import TerrainCostGrid2D


def _dijkstra_cost(problem: DiscreteProblem[tuple[int, int]]) -> float:
    graph = problem.graph
    costs = {problem.start: 0.0}
    queue = [(0.0, problem.start)]
    while queue:
        cost, node = heapq.heappop(queue)
        if cost != costs[node]:
            continue
        if node == problem.goal:
            return cost
        for neighbor in graph.neighbors(node):
            edge_cost = graph.edge_cost(node, neighbor)
            if not math.isfinite(edge_cost):
                continue
            candidate = cost + edge_cost
            if candidate < costs.get(neighbor, math.inf):
                costs[neighbor] = candidate
                heapq.heappush(queue, (candidate, neighbor))
    return math.inf


def _validate_path(problem: DiscreteProblem[tuple[int, int]], path: np.ndarray) -> float:
    graph = problem.graph
    total_cost = 0.0
    for point in path:
        assert graph.is_valid_node((int(point[0]), int(point[1])))
    for first, second in zip(path, path[1:]):
        source = (int(first[0]), int(first[1]))
        target = (int(second[0]), int(second[1]))
        edge_cost = graph.edge_cost(source, target)
        assert math.isfinite(edge_cost)
        total_cost += edge_cost
    return total_cost


def test_terrain_grid_integrates_terrain_over_touched_cells() -> None:
    graph = TerrainCostGrid2D([[1.0, 3.0], [5.0, 7.0]])

    assert graph.edge_cost((0, 0), (1, 0)) == 2.0
    assert math.isclose(graph.edge_cost((0, 0), (1, 1)), 4.0 * math.sqrt(2.0))
    assert math.isinf(graph.edge_cost((0, 0), (0, 0)))

    blocked_corner = TerrainCostGrid2D(
        [[1.0, 1.0], [1.0, 1.0]], occupancy=np.array([[False, True], [True, False]])
    )
    assert math.isinf(blocked_corner.edge_cost((0, 0), (1, 1)))


def test_jpsw_matches_dijkstra_on_seeded_weighted_grids() -> None:
    rng = np.random.default_rng(20261007)
    for _ in range(40):
        occupancy = rng.random((8, 10)) < 0.18
        occupancy[0, 0] = occupancy[7, 9] = False
        terrain = rng.uniform(0.25, 8.0, size=(8, 10))
        graph = TerrainCostGrid2D(terrain, occupancy=occupancy)
        problem = DiscreteProblem(graph, (0, 0), (9, 7))

        expected = _dijkstra_cost(problem)
        result = plan_jpsw(problem)
        assert result.success is math.isfinite(expected)
        if result.success:
            assert result.path is not None
            path_cost = _validate_path(problem, result.path)
            assert math.isclose(path_cost, expected, rel_tol=1e-10, abs_tol=1e-8)
            assert math.isclose(result.stats["path_cost"], path_cost, rel_tol=1e-10)
            assert result.stats["motion_checks"] > 0
            assert result.stats["neighborhood_checks"] > 0


def test_jpsw_handles_terrain_detours_and_uniform_cost_grids() -> None:
    terrain = np.ones((9, 11), dtype=float)
    terrain[2:7, 3:8] = 12.0
    graph = TerrainCostGrid2D(terrain)
    problem = DiscreteProblem(graph, (1, 4), (9, 4))

    result = plan_jpsw(problem)
    assert result.success
    assert result.path is not None
    assert np.any(result.path[:, 1] != 4.0)
    assert math.isclose(_validate_path(problem, result.path), _dijkstra_cost(problem))

    uniform = TerrainCostGrid2D(np.ones((9, 11), dtype=float))
    uniform_problem = DiscreteProblem(uniform, (1, 1), (9, 7))
    uniform_result = plan_jpsw(uniform_problem)
    assert uniform_result.success
    assert math.isclose(
        uniform_result.stats["path_cost"], _dijkstra_cost(uniform_problem), abs_tol=1e-8
    )


def test_jpsw_reports_unreachable_invalid_and_budget_stops() -> None:
    terrain = np.ones((6, 7), dtype=float)
    wall = {(3, y) for y in range(6)}
    graph = TerrainCostGrid2D(terrain, obstacles=wall)
    unreachable = plan_jpsw(DiscreteProblem(graph, (0, 0), (6, 5)))
    assert not unreachable.success
    assert unreachable.stop_reason is StopReason.NO_PROGRESS

    blocked_endpoint = plan_jpsw(DiscreteProblem(graph, (3, 0), (6, 5)))
    assert not blocked_endpoint.success
    assert blocked_endpoint.stop_reason is StopReason.NO_PROGRESS

    same_cell = plan_jpsw(DiscreteProblem(TerrainCostGrid2D(terrain), (2, 2), (2, 2)))
    assert same_cell.success
    assert same_cell.path is not None
    assert same_cell.path.tolist() == [[2.0, 2.0]]
    assert same_cell.stats["path_cost"] == 0.0

    budgeted = plan_jpsw(
        DiscreteProblem(TerrainCostGrid2D(np.ones((12, 12))), (1, 1), (10, 7)),
        params={"max_expansions": 1},
    )
    assert not budgeted.success
    assert budgeted.stop_reason is StopReason.MAX_ITERS

    with pytest.raises(ValueError, match="max_expansions"):
        plan_jpsw(
            DiscreteProblem(TerrainCostGrid2D(terrain), (0, 0), (6, 5)),
            params={"max_expansions": 0},
        )


def test_jpsw_rejects_spaces_outside_its_cost_contract() -> None:
    with pytest.raises(TypeError, match="TerrainCostGrid2D"):
        plan_jpsw(DiscreteProblem(Grid2DSearchSpace(5, 5), (0, 0), (4, 4)))

    class CustomTerrain(TerrainCostGrid2D):
        def edge_cost(self, first: tuple[int, int], second: tuple[int, int]) -> float:
            return super().edge_cost(first, second) * 2.0

    with pytest.raises(ValueError, match="built-in terrain edge-cost model"):
        plan_jpsw(DiscreteProblem(CustomTerrain(np.ones((5, 5))), (0, 0), (4, 4)))
