"""Runtime smoke tests for all supported 2D search planners."""

from __future__ import annotations

from pathplanning.search2d import PlanConfig, Planner, Search2D
from pathplanning.spaces.grid2d import Grid2DSearchSpace


def test_search2d_all_planners_smoke() -> None:
    planner = Search2D()
    cfg = PlanConfig(start=(5, 5), goal=(45, 25), graph=Grid2DSearchSpace())
    expected = {
        Planner.BFS,
        Planner.DFS,
        Planner.GREEDY_BEST_FIRST,
        Planner.DIJKSTRA,
        Planner.ASTAR,
        Planner.BIDIRECTIONAL_ASTAR,
        Planner.WEIGHTED_ASTAR,
        Planner.ANYTIME_ASTAR,
    }

    for algo in expected:
        result = planner.plan(algo, cfg)
        assert result.path is not None, f"{algo.value} returned no path"
        assert len(result.path) > 0, f"{algo.value} returned empty path"
        assert result.cost is not None
        assert result.nodes_expanded > 0
