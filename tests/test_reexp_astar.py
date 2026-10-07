"""Behavioral tests for Weighted A* with conditional re-expansion."""

from __future__ import annotations

from collections.abc import Iterable

import numpy as np
import pytest

from pathplanning.api import plan_discrete
from pathplanning.core.contracts import DiscreteProblem
from pathplanning.core.results import StopReason
from pathplanning.native import NativeGraph


class InconsistentHeuristicGraph:
    """Small directed graph where an improved CLOSED node changes the route."""

    _edges = {
        0: ((1, 3.0), (2, 1.0)),
        1: ((3, 2.0),),
        2: ((1, 1.0),),
        3: ((4, 10.0),),
        4: (),
    }
    _heuristic = {0: 4.0, 1: 0.0, 2: 3.0, 3: 0.0, 4: 0.0}

    def neighbors(self, node: int) -> Iterable[int]:
        return (neighbor for neighbor, _ in self._edges[node])

    def edge_cost(self, source: int, target: int) -> float:
        return next(cost for neighbor, cost in self._edges[source] if neighbor == target)

    def heuristic(self, node: int, _goal: int) -> float:
        return self._heuristic[node]


def _problem() -> DiscreteProblem[int]:
    return DiscreteProblem(graph=InconsistentHeuristicGraph(), start=0, goal=4)


def test_reexp_astar_reopens_closed_node_at_zero_threshold() -> None:
    result = plan_discrete(_problem(), planner="reexp_astar", params={"r": 0.0})

    assert result.success
    assert result.stop_reason is StopReason.SUCCESS
    assert result.stats["path_cost"] == pytest.approx(14.0)
    assert result.stats["reopens"] == 1.0
    assert result.path is not None
    assert np.array_equal(result.path[:, 0], np.array([0.0, 2.0, 1.0, 3.0, 4.0]))


@pytest.mark.parametrize(
    ("threshold", "mode", "expected_cost", "expected_reopens"),
    [
        (float("inf"), "abs", 15.0, 0.0),
        (1.0, "abs", 15.0, 0.0),
        (0.5, "abs", 14.0, 1.0),
        (0.5, "rel_edge", 14.0, 1.0),
        (0.3, "rel_g", 14.0, 1.0),
    ],
)
def test_reexp_astar_applies_conditional_threshold(
    threshold: float,
    mode: str,
    expected_cost: float,
    expected_reopens: float,
) -> None:
    result = plan_discrete(
        _problem(),
        planner="reexp_astar",
        params={"r": threshold, "r_mode": mode},
    )

    assert result.success
    assert result.stats["path_cost"] == pytest.approx(expected_cost)
    assert result.stats["reopens"] == expected_reopens


@pytest.mark.parametrize(
    ("params", "error", "message"),
    [
        ({"r": -0.1}, ValueError, "r must be >= 0.0"),
        ({"r_mode": "unknown"}, ValueError, "r_mode must be one of"),
        ({"tie_break": "unknown"}, ValueError, "tie_break must be one of"),
        ({"weight": 0.5}, ValueError, "weight must be >= 1.0"),
    ],
)
def test_reexp_astar_rejects_invalid_options(
    params: dict[str, object], error: type[Exception], message: str
) -> None:
    with pytest.raises(error, match=message):
        plan_discrete(_problem(), planner="reexp_astar", params=params)


def test_reexp_astar_honors_expansion_limit() -> None:
    result = plan_discrete(
        _problem(),
        planner="reexp_astar",
        params={"max_expansions": 1},
    )

    assert not result.success
    assert result.stop_reason is StopReason.MAX_ITERS
    assert result.iters == 1


def test_reexp_astar_honors_runtime_limit() -> None:
    node_count = 2_048
    graph = NativeGraph.from_edges(
        nodes=range(node_count),
        edges=((node, node + 1, 1.0) for node in range(node_count - 1)),
    )
    try:
        result = plan_discrete(
            DiscreteProblem(graph=graph, start=0, goal=node_count - 1),
            planner="reexp_astar",
            params={"max_runtime_ms": 0.01},
        )

        assert not result.success
        assert result.stop_reason is StopReason.TIME_BUDGET
        assert result.iters < node_count
    finally:
        graph.close()
