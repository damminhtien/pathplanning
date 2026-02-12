"""Headless BIT* interface over core continuous-space contracts."""

from __future__ import annotations

from collections.abc import Sequence

import numpy as np

from pathplanning.core.contracts import ContinuousSpace, GoalRegion, State
from pathplanning.core.params import RrtParams
from pathplanning.core.results import PlanResult
from pathplanning.planners.sampling.rrt_star import IndexFactory, RrtStarPlanner


class BitStar:
    """Contract-driven BIT* entrypoint.

    The implementation delegates to the shared RRT* engine to keep planner modules
    environment-agnostic and import-safe while retaining the public class surface.
    """

    def __init__(
        self,
        space: ContinuousSpace[State],
        params: RrtParams,
        rng: np.random.Generator,
        *,
        show_ellipse: bool = False,
        nn_index_factory: IndexFactory | None = None,
    ) -> None:
        self.space = space
        self.params = params.validate()
        self.rng = rng
        self.show_ellipse = show_ellipse
        self._delegate = RrtStarPlanner(
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


__all__ = ["BitStar"]
