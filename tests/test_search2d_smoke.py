"""Runtime smoke tests for 2D discrete planners via the public API."""

from __future__ import annotations

from pathplanning.api import plan_discrete
from pathplanning.core.contracts import DiscreteProblem
from pathplanning.registry import SEARCH_PLANNERS
from pathplanning.spaces.grid2d import Grid2DSearchSpace


def test_discrete_registered_planners_smoke() -> None:
    problem = DiscreteProblem(
        graph=Grid2DSearchSpace(),
        start=(5, 5),
        goal=(45, 25),
    )

    for planner_name in SEARCH_PLANNERS:
        result = plan_discrete(
            problem,
            planner=planner_name,
            params={"max_expansions": 50_000},
            seed=0,
        )
        assert result.path is not None, f"{planner_name} returned no path"
        assert result.path.shape[0] > 0, f"{planner_name} returned empty path"
        assert result.iters > 0
