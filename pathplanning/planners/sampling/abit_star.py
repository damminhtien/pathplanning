"""Headless ABIT* interface over core continuous-space contracts."""

from __future__ import annotations

from collections.abc import Mapping, Sequence

import numpy as np

from pathplanning.core.contracts import ContinuousProblem, ContinuousSpace, GoalRegion, State
from pathplanning.core.params import RrtParams
from pathplanning.core.results import PlanResult
from pathplanning.core.types import RNG
from pathplanning.planners.sampling._internal.problem_adapter import (
    coerce_rrt_params,
    resolve_rng,
)
from pathplanning.planners.sampling.rrt_star import IndexFactory, RrtStarPlanner


class AbitStar:
    """Contract-driven ABIT* entrypoint.

    This lightweight implementation keeps the module import-safe and
    environment-agnostic by delegating to the generic RRT* engine.
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


def plan_abit_star(
    problem: ContinuousProblem[State],
    *,
    params: RrtParams | Mapping[str, object] | None = None,
    rng: RNG | None = None,
) -> PlanResult:
    """Plan one ``ContinuousProblem`` with ABIT*."""
    resolved_params = coerce_rrt_params(problem, params)
    planner = AbitStar(problem.space, resolved_params, resolve_rng(rng))
    return planner.plan(problem.start, problem.goal)


__all__ = ["AbitStar", "plan_abit_star"]
