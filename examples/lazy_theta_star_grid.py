"""Run Lazy Theta* and inspect its deferred visibility checks."""

from __future__ import annotations

from pathplanning.api import plan_discrete
from pathplanning.core.contracts import DiscreteProblem
from pathplanning.spaces.grid2d import Grid2DSearchSpace


def main() -> None:
    obstacles = {(x, 9) for x in range(3, 21) if x != 12}
    graph = Grid2DSearchSpace(width=24, height=18, obstacles=obstacles)
    problem = DiscreteProblem(graph=graph, start=(1, 1), goal=(22, 16))
    result = plan_discrete(problem, planner="lazy_theta_star", params={"max_expansions": 1_000})

    print(f"success={result.success}, reason={result.stop_reason.value}")
    print(f"expanded={result.iters}, path_cost={result.stats['path_cost']:.3f}")
    print(f"line_of_sight_checks={result.stats['line_of_sight_checks']:.0f}")
    if result.path is not None:
        print(f"waypoints={len(result.path)}")
        print(result.path)


if __name__ == "__main__":
    main()
