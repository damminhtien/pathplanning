"""Correctness checks for Jump Point Search."""

from __future__ import annotations

import heapq
import math

import numpy as np

from pathplanning.core.contracts import DiscreteProblem
from pathplanning.core.results import StopReason
from pathplanning.planners.search.jump_point import plan_jps
from pathplanning.spaces.grid2d import Grid2DSearchSpace


def _problem(obstacles: set[tuple[int, int]]) -> DiscreteProblem[tuple[int, int]]:
    return DiscreteProblem(
        graph=Grid2DSearchSpace(width=24, height=18, obstacles=obstacles),
        start=(1, 1),
        goal=(22, 16),
    )


def _path_cost(path: np.ndarray) -> float:
    return float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum())


def _strict_grid_cost(problem: DiscreteProblem[tuple[int, int]]) -> float:
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
            dx, dy = neighbor[0] - node[0], neighbor[1] - node[1]
            if (
                dx
                and dy
                and (
                    not graph.is_valid_node((node[0] + dx, node[1]))
                    or not graph.is_valid_node((node[0], node[1] + dy))
                )
            ):
                continue
            candidate = cost + math.hypot(dx, dy)
            if candidate < costs.get(neighbor, math.inf):
                costs[neighbor] = candidate
                heapq.heappush(queue, (candidate, neighbor))
    return math.inf


def test_jps_matches_astar_cost_on_uniform_grid_maps() -> None:
    maps = [
        set(),
        {(x, 8) for x in range(3, 20) if x != 12},
        {(x, y) for x in range(7, 17) for y in range(4, 13) if (x, y) not in {(11, 8), (12, 8)}},
    ]
    for obstacles in maps:
        problem = _problem(obstacles)
        jps_result = plan_jps(problem)
        assert jps_result.success
        assert jps_result.stats["motion_checks"] > 0
        assert jps_result.stats["jps_expanded"] == jps_result.iters
        assert math.isclose(_path_cost(jps_result.path), _strict_grid_cost(problem), abs_tol=1e-8)
        for x, y in jps_result.path:
            assert problem.graph.is_valid_node((int(x), int(y)))
        for first, second in zip(jps_result.path, jps_result.path[1:]):
            dx, dy = int(second[0] - first[0]), int(second[1] - first[1])
            assert max(abs(dx), abs(dy)) == 1
            if dx and dy:
                assert problem.graph.is_valid_node((int(first[0] + dx), int(first[1])))
                assert problem.graph.is_valid_node((int(first[0]), int(first[1] + dy)))


def test_jps_matches_strict_dijkstra_on_seeded_random_maps() -> None:
    rng = np.random.default_rng(20261007)
    for _ in range(150):
        occupancy = rng.random((8, 10)) < 0.18
        occupancy[0, 0] = occupancy[7, 9] = False
        graph = Grid2DSearchSpace(width=10, height=8, occupancy=occupancy)
        problem = DiscreteProblem(graph=graph, start=(0, 0), goal=(9, 7))
        result = plan_jps(problem)
        expected = _strict_grid_cost(problem)
        assert result.success is math.isfinite(expected)
        if result.success:
            assert math.isclose(_path_cost(result.path), expected, abs_tol=1e-8)


def test_jps_handles_unreachable_and_invalid_endpoints() -> None:
    wall = {(2, y) for y in range(5)}
    graph = Grid2DSearchSpace(width=5, height=5, obstacles=wall)
    unreachable = plan_jps(DiscreteProblem(graph, (0, 0), (4, 4)))
    assert not unreachable.success
    assert unreachable.stop_reason is StopReason.NO_PROGRESS

    blocked_start = plan_jps(DiscreteProblem(graph, (2, 2), (4, 4)))
    assert not blocked_start.success
    assert blocked_start.stop_reason is StopReason.NO_PROGRESS


def test_jps_start_equals_goal_and_budget_validation() -> None:
    graph = Grid2DSearchSpace(width=5, height=5)
    same_cell = plan_jps(DiscreteProblem(graph, (2, 2), (2, 2)))
    assert same_cell.success
    assert same_cell.path.tolist() == [[2.0, 2.0]]
    assert same_cell.stats["path_cost"] == 0.0
    assert same_cell.stats["motion_checks"] == 0.0

    try:
        plan_jps(DiscreteProblem(graph, (0, 0), (4, 4)), params={"max_expansions": 0})
    except ValueError as error:
        assert "max_expansions" in str(error)
    else:
        raise AssertionError("zero expansion budget should be rejected")


def test_jps_rejects_weighted_grid_subclasses() -> None:
    class WeightedGrid(Grid2DSearchSpace):
        def edge_cost(self, first: tuple[int, int], second: tuple[int, int]) -> float:
            return super().edge_cost(first, second) * 2.0

    graph = WeightedGrid(width=5, height=5)
    try:
        plan_jps(DiscreteProblem(graph, (0, 0), (4, 4)))
    except ValueError as error:
        assert "uniform" in str(error)
    else:
        raise AssertionError("JPS should reject weighted edge costs")
