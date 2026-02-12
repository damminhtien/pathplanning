"""Run a headless demo of all legacy 2D search algorithms."""

from __future__ import annotations

from pathplanning.planners.search.plan2d_facade import (
    Heuristic,
    PlanConfig,
    Planner,
    Search2dFacade,
)

from ._legacy2d_common import OpenGridLegacyGraph

ALL_ALGORITHMS = [
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
]


def demo() -> None:
    planner = Search2dFacade()
    cfg = PlanConfig(
        s_start=(5, 5),
        s_goal=(45, 25),
        graph=OpenGridLegacyGraph(),
        heuristic=Heuristic.EUCLIDEAN,
    )

    for algo in ALL_ALGORITHMS:
        res = planner.plan(algo, cfg)
        cost = f"{res.cost:.3f}" if res.cost is not None else "None"
        print(
            f"{algo.value:>24} | "
            f"nodes={res.nodes_expanded:>4} | "
            f"cost={cost:>8} | "
            f"path_len={len(res.path) if res.path else 0} | "
            f"{res.runtime_s:.4f}s"
        )


if __name__ == "__main__":
    demo()
