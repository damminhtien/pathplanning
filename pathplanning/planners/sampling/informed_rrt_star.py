"""Thin Python API for the native C Informed RRT* planner."""

from __future__ import annotations

from collections.abc import Mapping, Sequence

import numpy as np

from pathplanning.core.contracts import ContinuousProblem, ContinuousSpace, GoalRegion, State
from pathplanning.core.params import RrtParams
from pathplanning.core.results import PlanResult
from pathplanning.core.trace import TraceOptions
from pathplanning.core.types import RNG
from pathplanning.native.continuous import run_native_continuous
from pathplanning.nn.index import NearestNeighborIndex
from pathplanning.planners.sampling._internal.continuous import validate_objective
from pathplanning.planners.sampling._internal.problem_adapter import (
    coerce_rrt_params,
    resolve_rng,
)
from pathplanning.planners.sampling.rrt_star import IndexFactory, RrtStarPlanner


class InformedRrtStar(RrtStarPlanner):
    """Object wrapper that selects direct informed sampling in the C core."""

    def __init__(
        self,
        space: ContinuousSpace[State],
        params: RrtParams,
        rng: np.random.Generator,
        nn_index_factory: IndexFactory | None = None,
    ) -> None:
        super().__init__(space, params, rng, nn_index_factory, objective=None)

    def plan(
        self,
        start: Sequence[float] | State,
        goal_region: GoalRegion[State],
        *,
        trace: TraceOptions | None = None,
    ) -> PlanResult:
        return run_native_continuous(
            self.space,
            start,
            goal_region,
            self.params,
            self.rng,
            planner="informed_rrt_star",
            trace=trace,
        )


def plan_informed_rrt_star(
    problem: ContinuousProblem[State],
    *,
    params: RrtParams | Mapping[str, object] | None = None,
    rng: RNG | None = None,
    trace: TraceOptions | None = None,
) -> PlanResult:
    """Plan with native Informed RRT* for additive Euclidean path length."""
    validate_objective(problem.objective, "Informed RRT*")
    resolved_params = coerce_rrt_params(problem, params)
    return run_native_continuous(
        problem.space,
        problem.start,
        problem.goal,
        resolved_params,
        resolve_rng(rng),
        planner="informed_rrt_star",
        trace=trace,
    )


__all__ = ["InformedRrtStar", "NearestNeighborIndex", "plan_informed_rrt_star"]
