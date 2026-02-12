"""Representative runtime smoke tests across planning families."""

from __future__ import annotations

import numpy as np

from pathplanning.planners.sampling.rrt_grid2d import Rrt
from pathplanning.planners.search.astar_3d import Weighted_A_star


def test_sampling2d_rrt_runtime_smoke() -> None:
    np.random.seed(0)
    planner = Rrt((2, 2), (49, 24), step_len=0.5, goal_sample_rate=0.05, iter_max=200)
    result = planner.planning()
    assert result is None or isinstance(result, list)


def test_search3d_astar_runtime_smoke() -> None:
    planner = Weighted_A_star(resolution=1.0)
    # Limit expansions to keep test fast and headless.
    ok = planner.run(N=200)
    assert ok is True
