"""Legacy BIT* interface; the implementation currently delegates to RRT*."""

from __future__ import annotations

from collections.abc import Mapping, Sequence

import numpy as np

from pathplanning.core.contracts import (
    ContinuousProblem,
    ContinuousSpace,
    GoalRegion,
    Objective,
    State,
)
from pathplanning.core.params import RrtParams
from pathplanning.core.results import PlanResult
from pathplanning.core.types import RNG
from pathplanning.planners.sampling._internal.problem_adapter import (
    coerce_rrt_params,
    resolve_rng,
)
from pathplanning.planners.sampling.rrt_star import IndexFactory, RrtStarPlanner


class BitStar:
    """Compatibility wrapper that executes the shared RRT* implementation.

    This class does not implement BIT* and is not registered as a supported planner.
    """

    def __init__(
        self,
        space: ContinuousSpace[State],
        params: RrtParams,
        rng: np.random.Generator,
        *,
        show_ellipse: bool = False,
        nn_index_factory: IndexFactory | None = None,
        objective: Objective[State] | None = None,
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
            objective=objective,
        )

    def plan(self, start: Sequence[float] | State, goal_region: GoalRegion[State]) -> PlanResult:
        """Plan with RRT* through the legacy BIT* wrapper."""
        return self._delegate.plan(start, goal_region)

    def run(self, start: Sequence[float] | State, goal_region: GoalRegion[State]) -> PlanResult:
        """Backward-compatible alias for ``plan``."""
        return self.plan(start, goal_region)


def plan_bit_star(
    problem: ContinuousProblem[State],
    *,
    params: RrtParams | Mapping[str, object] | None = None,
    rng: RNG | None = None,
) -> PlanResult:
    """Compatibility entry point that plans with RRT*, not BIT*."""
    resolved_params = coerce_rrt_params(problem, params)
    planner = BitStar(
        problem.space,
        resolved_params,
        resolve_rng(rng),
        objective=problem.objective,
    )
    return planner.plan(problem.start, problem.goal)


__all__ = ["BitStar", "plan_bit_star"]
