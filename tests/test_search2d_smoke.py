"""Runtime smoke tests for all supported 2D search planners."""

from __future__ import annotations

from pathplanning import PlanConfig, Planner, Search2D


def test_search2d_all_planners_smoke() -> None:
    planner = Search2D()
    cfg = PlanConfig(s_start=(5, 5), s_goal=(45, 25))
    expected = {
        Planner.BREADTH_FIRST_SEARCH,
        Planner.DEPTH_FIRST_SEARCH,
        Planner.BEST_FIRST_SEARCH,
        Planner.DIJKSTRA,
        Planner.ASTAR,
        Planner.BIDIRECTIONAL_ASTAR,
        Planner.ANYTIME_DSTAR,
        Planner.ANYTIME_REPAIRING_ASTAR,
        Planner.LIFELONG_PLANNING_ASTAR,
        Planner.REALTIME_ADAPTIVE_ASTAR,
        Planner.LEARNING_REALTIME_ASTAR,
        Planner.DSTAR_LITE,
        Planner.DSTAR,
    }

    for algo in expected:
        result = planner.plan(algo, cfg)
        assert result.path is not None, f"{algo.value} returned no path"
        assert len(result.path) > 0, f"{algo.value} returned empty path"
        assert result.cost is not None
        assert result.nodes_expanded is not None
