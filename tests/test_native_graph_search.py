"""Tests for native CSR graph initialization and search execution."""

from __future__ import annotations

from collections.abc import Iterable

import numpy as np
import pytest

from pathplanning.api import plan_discrete
from pathplanning.core.contracts import DiscreteProblem
from pathplanning.native import NativeGraph, NativeGraphError
from pathplanning.native._ffi import load_native_library
from pathplanning.planners.search._internal.native import NativeSearchError
from pathplanning.spaces.grid2d import Grid2DSearchSpace


class CountingGraph:
    def __init__(self) -> None:
        self.adjacency = {
            0: [(1, 5.0), (2, 1.0)],
            1: [(3, 1.0)],
            2: [(1, 1.0), (3, 20.0)],
            3: [],
        }
        self.neighbor_calls: dict[int, int] = {}
        self.edge_cost_calls = 0
        self.heuristic_calls = 0

    def neighbors(self, node: int) -> Iterable[int]:
        self.neighbor_calls[node] = self.neighbor_calls.get(node, 0) + 1
        return (neighbor for neighbor, _ in self.adjacency[node])

    def edge_cost(self, source: int, target: int) -> float:
        self.edge_cost_calls += 1
        return next(cost for neighbor, cost in self.adjacency[source] if neighbor == target)

    def heuristic(self, node: int, goal: int) -> float:
        self.heuristic_calls += 1
        return float(abs(goal - node))


def test_python_graph_callbacks_run_only_during_one_time_snapshot() -> None:
    graph = CountingGraph()
    problem = DiscreteProblem(graph=graph, start=0, goal=3)

    result = plan_discrete(problem, planner="astar")

    assert result.success
    assert result.stats["path_cost"] == pytest.approx(3.0)
    assert np.array_equal(result.path[:, 0], np.array([0.0, 2.0, 1.0, 3.0]))
    assert graph.neighbor_calls == {0: 1, 1: 1, 2: 1, 3: 1}
    assert graph.edge_cost_calls == 5
    assert graph.heuristic_calls == 4
    assert result.stats["elapsed_s"] >= result.stats["native_search_s"]
    assert result.stats["graph_init_s"] > 0.0
    assert result.stats["native_search_s"] > 0.0


def test_goal_predicate_is_resolved_before_native_search() -> None:
    graph = CountingGraph()

    class CountingGoal:
        calls = 0

        def is_goal(self, node: int) -> bool:
            self.calls += 1
            return node == 3

    goal = CountingGoal()
    result = plan_discrete(
        DiscreteProblem(graph=graph, start=0, goal=goal),
        planner="bfs",
    )

    assert result.success
    assert goal.calls == 4
    assert graph.neighbor_calls == {0: 1, 1: 1, 2: 1, 3: 1}
    assert graph.edge_cost_calls == 5


def test_unreachable_exact_goal_does_not_expand_its_component() -> None:
    class DisconnectedGraph:
        def __init__(self) -> None:
            self.calls: list[int] = []

        def neighbors(self, node: int) -> Iterable[int]:
            self.calls.append(node)
            return {0: (1,), 1: (), 99: (100,), 100: ()}[node]

        def edge_cost(self, source: int, target: int) -> float:
            _ = (source, target)
            return 1.0

    graph = DisconnectedGraph()
    result = plan_discrete(
        DiscreteProblem(graph=graph, start=0, goal=99),
        planner="bfs",
    )

    assert not result.success
    assert graph.calls == [0, 1]


def test_native_graph_reuses_csr_and_supports_directed_bidirectional_search() -> None:
    graph = NativeGraph.from_edges(
        nodes=[0, 1, 2, 3],
        edges=[(0, 1, 2.0), (1, 2, 3.0)],
    )
    problem = DiscreteProblem(graph=graph, start=0, goal=2)

    forward = plan_discrete(problem, planner="bidirectional_dijkstra")
    backward = plan_discrete(
        DiscreteProblem(graph=graph, start=2, goal=0),
        planner="bidirectional_dijkstra",
    )

    assert forward.success
    assert forward.stats["path_cost"] == pytest.approx(5.0)
    assert np.array_equal(forward.path[:, 0], np.array([0.0, 1.0, 2.0]))
    assert not backward.success


def test_bidirectional_astar_uses_consistent_heuristic_and_preserves_optimal_cost() -> None:
    class HeuristicGraph:
        adjacency = {
            0: [(1, 1.0), (2, 1.0), (3, 5.0)],
            1: [(3, 1.0)],
            2: [(3, 1.0)],
            3: [],
        }
        heuristic_values = {0: 2.0, 1: 1.0, 2: 1.0, 3: 0.0}

        def neighbors(self, node: int) -> tuple[int, ...]:
            return tuple(target for target, _ in self.adjacency[node])

        def edge_cost(self, source: int, target: int) -> float:
            return next(cost for neighbor, cost in self.adjacency[source] if neighbor == target)

        def heuristic(self, node: int, _goal: int) -> float:
            return self.heuristic_values[node]

    result = plan_discrete(
        DiscreteProblem(graph=HeuristicGraph(), start=0, goal=3),
        planner="bidirectional_astar",
    )

    assert result.success
    assert result.stats["path_cost"] == pytest.approx(2.0)
    assert result.path is not None
    assert result.path[0, 0] == 0.0
    assert result.path[-1, 0] == 3.0


def test_bidirectional_astar_rejects_inconsistent_heuristics() -> None:
    class InconsistentGraph:
        adjacency = {0: [(1, 1.0)], 1: [(2, 1.0)], 2: []}
        heuristic_values = {0: 0.0, 1: 10.0, 2: 0.0}

        def neighbors(self, node: int) -> tuple[int, ...]:
            return tuple(target for target, _ in self.adjacency[node])

        def edge_cost(self, source: int, target: int) -> float:
            return next(cost for neighbor, cost in self.adjacency[source] if neighbor == target)

        def heuristic(self, node: int, _goal: int) -> float:
            return self.heuristic_values[node]

    with pytest.raises(NativeSearchError, match="consistent heuristic"):
        plan_discrete(
            DiscreteProblem(graph=InconsistentGraph(), start=0, goal=2),
            planner="bidirectional_astar",
        )


def test_bidirectional_dijkstra_requires_an_exact_goal() -> None:
    class GoalPredicate:
        def is_goal(self, node: int) -> bool:
            return node == 1

    graph = NativeGraph.from_edges(nodes=[0, 1], edges=[(0, 1, 1.0)])

    with pytest.raises(ValueError, match="requires an exact goal node"):
        plan_discrete(
            DiscreteProblem(graph=graph, start=0, goal=GoalPredicate()),
            planner="bidirectional_dijkstra",
        )


def test_builtin_grid_constructs_native_adjacency_without_python_edge_calls() -> None:
    class CountingGrid(Grid2DSearchSpace):
        def __init__(self) -> None:
            super().__init__(width=5, height=3, obstacles={(2, 1)})
            self.neighbor_calls = 0
            self.edge_cost_calls = 0

        def neighbors(self, node: tuple[int, int]) -> Iterable[tuple[int, int]]:
            self.neighbor_calls += 1
            return super().neighbors(node)

        def edge_cost(self, source: tuple[int, int], target: tuple[int, int]) -> float:
            self.edge_cost_calls += 1
            return super().edge_cost(source, target)

    graph = CountingGrid()
    result = plan_discrete(
        DiscreteProblem(graph=graph, start=(0, 1), goal=(4, 1)),
        planner="astar",
    )

    assert result.success
    assert graph.neighbor_calls == 0
    assert graph.edge_cost_calls == 0
    assert (2.0, 1.0) not in map(tuple, result.path.tolist())


def test_native_graph_accepts_csr_arrays_and_custom_labels() -> None:
    graph = NativeGraph.from_csr(
        np.array([0, 1, 1], dtype=np.int32),
        np.array([1], dtype=np.int32),
        np.array([2.0]),
        node_labels=[(0, 0), (1, 0)],
    )

    result = plan_discrete(
        DiscreteProblem(graph=graph, start=(0, 0), goal=(1, 0)),
        planner="dijkstra",
    )

    assert result.success
    assert result.stats["path_cost"] == pytest.approx(2.0)
    assert np.array_equal(result.path, np.array([[0.0, 0.0], [1.0, 0.0]]))


def test_native_graph_uses_implicit_integer_labels_and_exact_goal() -> None:
    graph = NativeGraph.from_csr([0, 1, 1], [1], [2.0])

    result = plan_discrete(
        DiscreteProblem(graph=graph, start=0, goal=1),
        planner="dijkstra",
    )

    assert result.success
    assert np.array_equal(result.path[:, 0], np.array([0.0, 1.0]))
    assert result.stats["path_cost"] == pytest.approx(2.0)


@pytest.mark.parametrize("planner", ["astar", "bidirectional_dijkstra"])
def test_native_graph_missing_exact_goal_is_unreachable(planner: str) -> None:
    graph = NativeGraph.from_csr([0, 1, 1], [1], [1.0])

    result = plan_discrete(
        DiscreteProblem(graph=graph, start=0, goal=99),
        planner=planner,
    )

    assert not result.success
    assert result.path is None


def test_native_graph_validates_csr_shape_and_endpoints() -> None:
    with pytest.raises(NativeGraphError, match="end at the edge count"):
        NativeGraph.from_csr([0, 2, 2], [1], [1.0])

    with pytest.raises(NativeGraphError, match="outside the graph"):
        NativeGraph.from_csr([0, 1], [1], [1.0])


def test_search_engine_exports_only_native_graph_planning_entrypoints() -> None:
    library = load_native_library()

    assert hasattr(library, "pp_native_search_plan")
    assert not hasattr(library, "pp_search_plan")
    assert not hasattr(library, "pp_astar_plan")
