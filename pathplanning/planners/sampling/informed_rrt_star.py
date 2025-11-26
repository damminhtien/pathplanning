"""Headless informed sampler wrapper over core continuous-space contracts."""

from __future__ import annotations

from collections.abc import Mapping, Sequence

import numpy as np

from pathplanning.core.contracts import ContinuousProblem, ContinuousSpace, GoalRegion, State
from pathplanning.core.params import RrtParams
from pathplanning.core.results import PlanResult
from pathplanning.core.types import RNG
from pathplanning.nn.index import NearestNeighborIndex
from pathplanning.planners.sampling._internal.problem_adapter import (
    coerce_rrt_params,
    resolve_rng,
)
from pathplanning.planners.sampling.rrt_star import IndexFactory, RrtStarPlanner


class InformedRrtStar:
    """Contract-driven Informed RRT* interface.

    This implementation keeps the module import-safe and environment-agnostic by
    delegating planning to the generic ``RrtStarPlanner``. It preserves the
    class-level API while removing hard dependencies on legacy environment files.
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
        """Plan a path by informed sampling over the provided contracts."""
        return self._delegate.plan(start, goal_region)

    def informed_rrt(
        self,
        start: Sequence[float] | State,
        goal_region: GoalRegion[State],
    ) -> PlanResult:
        """Backward-compatible alias for ``plan``."""
        return self.plan(start, goal_region)


def plan_informed_rrt_star(
    problem: ContinuousProblem[State],
    *,
    params: RrtParams | Mapping[str, object] | None = None,
    rng: RNG | None = None,
) -> PlanResult:
    """Plan one ``ContinuousProblem`` with Informed RRT*."""
    resolved_params = coerce_rrt_params(problem, params)
    planner = InformedRrtStar(problem.space, resolved_params, resolve_rng(rng))
    return planner.plan(problem.start, problem.goal)


__all__ = ["InformedRrtStar", "NearestNeighborIndex", "plan_informed_rrt_star"]
