"""Correctness checks for Jump Point Search."""

from __future__ import annotations

import heapq
import math

import numpy as np

from pathplanning.core.contracts import DiscreteProblem
from pathplanning.planners.search._grid_utils import _native_valid_nodes
from pathplanning.planners.search.jump_point import plan_jps
from pathplanning.spaces.grid2d import Grid2DSearchSpace


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


def test_native_valid_mask_matches_occupancy_blocked_cells_and_callback() -> None:
    occupancy = np.array(
        [[False, True, False], [False, False, False]],
        dtype=bool,
    )
    graph = Grid2DSearchSpace(width=3, height=2, occupancy=occupancy, obstacles={(2, 0)})
    graph.set_blocked_predicate(lambda cell: cell == (0, 1))

    actual = _native_valid_nodes(graph).reshape((2, 3)).astype(bool)
    expected = np.array(
        [[graph.is_valid_node((x, y)) for x in range(graph.x_range)] for y in range(graph.y_range)],
        dtype=bool,
    )
    np.testing.assert_array_equal(actual, expected)

    class CustomValidityGrid(Grid2DSearchSpace):
        def is_valid_node(self, cell: tuple[int, int]) -> bool:
            return super().is_valid_node(cell) and cell != (1, 1)

    custom_graph = CustomValidityGrid(width=3, height=2, occupancy=occupancy)
    custom_actual = _native_valid_nodes(custom_graph).reshape((2, 3)).astype(bool)
    custom_expected = np.array(
        [
            [custom_graph.is_valid_node((x, y)) for x in range(custom_graph.x_range)]
            for y in range(custom_graph.y_range)
        ],
        dtype=bool,
    )
    np.testing.assert_array_equal(custom_actual, custom_expected)
