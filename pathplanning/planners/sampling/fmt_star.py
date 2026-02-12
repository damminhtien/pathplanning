"""Headless FMT* interface over core continuous-space contracts."""

from __future__ import annotations

from collections.abc import Sequence

import numpy as np

from pathplanning.core.contracts import ContinuousSpace, GoalRegion, State
from pathplanning.core.params import RrtParams
from pathplanning.core.results import PlanResult
from pathplanning.planners.sampling.rrt import IndexFactory, RrtPlanner


class FmtStar:
    """Contract-driven FMT* entrypoint.

    This lightweight implementation reuses the generic RRT planner backend so
    planner modules depend only on abstract space/goal interfaces.
    """

    def __init__(
        self,
        space: ContinuousSpace[State],
        params: RrtParams,
        rng: np.random.Generator,
        *,
        nn_index_factory: IndexFactory | None = None,
    ) -> None:
        self.space = space
        self.params = params.validate()
        self.rng = rng
        self._delegate = RrtPlanner(
            space=space,
            params=self.params,
            rng=rng,
            nn_index_factory=nn_index_factory,
        )

    def plan(self, start: Sequence[float] | State, goal_region: GoalRegion[State]) -> PlanResult:
        """Plan a path using contract-based sampling interfaces."""
        return self._delegate.plan(start, goal_region)

    def run(self, start: Sequence[float] | State, goal_region: GoalRegion[State]) -> PlanResult:
        """Backward-compatible alias for ``plan``."""
        return self.plan(start, goal_region)


__all__ = ["FmtStar"]
