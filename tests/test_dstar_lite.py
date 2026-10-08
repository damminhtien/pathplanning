"""D* Lite incremental-repair correctness tests."""

from __future__ import annotations

from collections.abc import Iterable
import math
from math import inf

import pytest

from pathplanning import plan_discrete
from pathplanning.core.contracts import DiscreteProblem
from pathplanning.core.results import StopReason
from pathplanning.native import NativeGraph
from pathplanning.planners.search.dstar_lite import DStarLitePlanner
from pathplanning.spaces.grid2d import Grid2DSearchSpace


class WeightedGraph:
    def __init__(self) -> None:
        self.edges = {
            (0, 0): {(1, 0): 1.0, (0, 1): 2.0},
            (1, 0): {(2, 0): 1.0},
            (0, 1): {(2, 0): 2.0},
            (2, 0): {},
        }

    def neighbors(self, node: tuple[int, int]) -> Iterable[tuple[int, int]]:
        return self.edges.get(node, {})

    def edge_cost(self, first: tuple[int, int], second: tuple[int, int]) -> float:
        return self.edges.get(first, {}).get(second, float("inf"))

    def heuristic(self, first: tuple[int, int], second: tuple[int, int]) -> float:
        return 0.0


def test_dstar_lite_repairs_after_edge_cost_increases_and_moves_start() -> None:
    graph = WeightedGraph()
    planner = DStarLitePlanner(graph, (0, 0), (2, 0))

    initial = planner.plan()
    assert initial.success
    assert initial.stats["path_cost"] == 2.0
    assert initial.path.tolist() == [[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]]

    planner.update_edges([((0, 0), (1, 0), float("inf"))])
    repaired = planner.plan()
    assert repaired.success
    assert repaired.stats["path_cost"] == 4.0
    assert repaired.path.tolist() == [[0.0, 0.0], [0.0, 1.0], [2.0, 0.0]]

    planner.move_start((0, 1))
    moved = planner.plan()
    assert moved.success
    assert moved.stats["path_cost"] == 2.0

    planner.update_edges([((0, 0), (1, 0), None)])
    planner.reset(start=(0, 0), goal=(2, 0))
    restored = planner.plan()
    assert restored.success
    assert restored.stats["path_cost"] == 2.0
    planner.reset(start=(2, 0), goal=(2, 0))
    trivial = planner.plan()
    assert trivial.success
    assert trivial.path is not None
    assert trivial.path.tolist() == [[2.0, 0.0]]
    assert trivial.stats["path_cost"] == 0.0
    planner.close()


def test_dstar_lite_grid_adapter_keeps_blocked_edges_updateable() -> None:
    graph = Grid2DSearchSpace(3, 2, obstacles={(1, 0)})
    problem = DiscreteProblem(graph, (0, 0), (2, 0))

    with pytest.raises(ValueError, match="valid grid cells"):
        DStarLitePlanner(graph, (1, 0), (2, 0))

    initial = plan_discrete(problem, planner="dstar_lite")
    assert initial.success
    assert math.isclose(initial.stats["path_cost"], 2 * math.sqrt(2))

    planner = DStarLitePlanner(graph, (0, 0), (2, 0))
    planner.update_edges([((0, 0), (1, 0), 1.0), ((1, 0), (2, 0), 1.0)])
    opened = planner.plan()
    assert opened.success
    assert opened.stats["path_cost"] == 2.0

    planner.update_edges([((0, 0), (1, 0), inf), ((1, 0), (2, 0), inf)])
    blocked = planner.plan()
    assert blocked.success
    assert math.isclose(blocked.stats["path_cost"], 2 * math.sqrt(2))
    planner.move_start((0, 1))
    moved = planner.plan()
    assert moved.success
    assert moved.path is not None and moved.path[0].tolist() == [0.0, 1.0]
    assert math.isclose(moved.stats["path_cost"], 1 + math.sqrt(2))
    planner.close()

    budgeted = plan_discrete(
        DiscreteProblem(WeightedGraph(), (0, 0), (2, 0)),
        planner="dstar_lite",
        params={"max_expansions": 1},
    )
    assert not budgeted.success
    assert budgeted.stop_reason is StopReason.MAX_ITERS

    disconnected = NativeGraph.from_edges(nodes=(0, 1), edges=((0, 1, inf),))
    no_path = plan_discrete(DiscreteProblem(disconnected, 0, 1), planner="dstar_lite")
    assert not no_path.success
    assert no_path.stop_reason is StopReason.NO_PROGRESS

    timed = plan_discrete(
        DiscreteProblem(NativeGraph.from_edges((0, 1), ((0, 1, 1.0),)), 0, 1),
        planner="dstar_lite",
        params={"max_runtime_ms": 1e-12},
    )
    assert not timed.success
    assert timed.stop_reason is StopReason.TIME_BUDGET
