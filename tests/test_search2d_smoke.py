"""Runtime smoke tests for 2D discrete planners via the public API."""

from __future__ import annotations

import numpy as np

from pathplanning.api import plan_discrete
from pathplanning.core.contracts import DiscreteProblem
from pathplanning.registry import list_planners
from pathplanning.spaces.grid2d import Grid2DSearchSpace
from pathplanning.spaces.terrain_grid2d import TerrainCostGrid2D


def test_discrete_registered_planners_smoke() -> None:
    for planner_name in list_planners("discrete"):
        graph = (
            TerrainCostGrid2D(np.ones((30, 50), dtype=np.float64))
            if planner_name == "jpsw"
            else Grid2DSearchSpace()
        )
        problem = DiscreteProblem(graph=graph, start=(5, 5), goal=(45, 25))
        result = plan_discrete(
            problem,
            planner=planner_name,
            params={"max_expansions": 50_000},
            seed=0,
        )
        assert result.path is not None, f"{planner_name} returned no path"
        assert result.path.shape[0] > 0, f"{planner_name} returned empty path"
        assert result.iters > 0
