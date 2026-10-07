"""Run Weighted Jump Point Search on a terrain-cost grid."""

from __future__ import annotations

import numpy as np

from pathplanning.api import plan_discrete
from pathplanning.core.contracts import DiscreteProblem
from pathplanning.spaces.terrain_grid2d import TerrainCostGrid2D


def main() -> None:
    terrain = np.ones((18, 24), dtype=float)
    terrain[4:14, 8:16] = 5.0
    obstacles = {(x, 8) for x in range(3, 20) if x != 12}
    graph = TerrainCostGrid2D(terrain, obstacles=obstacles)
    problem = DiscreteProblem(graph=graph, start=(1, 1), goal=(22, 16))
    result = plan_discrete(problem, planner="jpsw", params={"max_expansions": 1_000})

    print(f"success={result.success}, reason={result.stop_reason.value}")
    print(f"expanded_jump_points={result.iters}, path_cost={result.stats['path_cost']:.3f}")
    if result.path is not None:
        print(f"traversed_cells={len(result.path)}")


if __name__ == "__main__":
    main()
