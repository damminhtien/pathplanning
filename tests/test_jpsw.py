"""Correctness checks for Weighted Jump Point Search."""

from __future__ import annotations

import heapq
import math

import numpy as np

from pathplanning.core.contracts import DiscreteProblem
from pathplanning.planners.search.jump_point_weighted import plan_jpsw
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
