"""
Run a demo of the Search2D planner with all algorithms.

This script imports the necessary classes and functions from the `plan2d_facade` module,
defines a list of all algorithms available for pathfinding, and runs a demo that executes
each algorithm, printing the results including nodes expanded, cost, path length, and runtime.

author: damminhtien
"""
try:
    from .plan2d_facade import Search2dFacade, PlanConfig, Heuristic, Planner
except ImportError:  # pragma: no cover - script execution fallback
    from plan2d_facade import Search2dFacade, PlanConfig, Heuristic, Planner

ALL_ALGORITHMS = [
    # Uninformed Search
    Planner.BREADTH_FIRST_SEARCH,
    Planner.DEPTH_FIRST_SEARCH,
    # Best-First & Dijkstra
    Planner.BEST_FIRST_SEARCH,
    Planner.DIJKSTRA,
    # A* Variants
    Planner.ASTAR,
    Planner.BIDIRECTIONAL_ASTAR,
    Planner.ANYTIME_DSTAR,
    Planner.ANYTIME_REPAIRING_ASTAR,
    Planner.LIFELONG_REPAIRING_ASTAR,
    Planner.REALTIME_ADAPTIVE_ASTAR,
    Planner.LEARNING_REALTIME_ASTAR,
    Planner.DSTAR_LITE,
    Planner.DSTAR,
]


def demo():
    """ Run a demo of the Search2D planner with all algorithms. """
    planner = Search2dFacade()
    cfg = PlanConfig(
        s_start=(5, 5),
        s_goal=(45, 25),
        heuristic=Heuristic.EUCLIDEAN)

    for algo in ALL_ALGORITHMS:
        res = planner.plan(algo, cfg)
        print(
            f"{algo.value:>18} | "
            f"nodes={res.nodes_expanded:>4} | "
            f"cost={res.cost:.3f} | "
            f"path_len={len(res.path) if res.path else 0} | "
            f"{res.runtime_s:.4f}s"
        )


if __name__ == "__main__":
    demo()
