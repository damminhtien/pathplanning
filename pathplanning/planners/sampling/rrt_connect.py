"""Thin Python API for native bidirectional RRT-Connect."""

from __future__ import annotations

from collections.abc import Mapping, Sequence

import numpy as np

from pathplanning.core.contracts import ContinuousProblem, ContinuousSpace, GoalRegion, State
from pathplanning.core.params import RrtParams
from pathplanning.core.results import PlanResult
from pathplanning.core.trace import TraceOptions
from pathplanning.core.types import RNG
from pathplanning.native.continuous import run_native_continuous
from pathplanning.planners.sampling._internal.problem_adapter import (
    coerce_rrt_params,
    resolve_rng,
)


class RrtConnect:
    """Object wrapper for the C two-tree RRT-Connect planner."""

    def __init__(
        self,
        space: ContinuousSpace[State],
        params: RrtParams,
        rng: np.random.Generator,
    ) -> None:
        self.space = space
        self.params = params.validate()
        self.rng = rng

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
            planner="rrt_connect",
            trace=trace,
        )


def plan_rrt_connect(
    problem: ContinuousProblem[State],
    *,
    params: RrtParams | Mapping[str, object] | None = None,
    rng: RNG | None = None,
    trace: TraceOptions | None = None,
) -> PlanResult:
    """Plan a point-to-point problem with native RRT-Connect."""
    resolved_params = coerce_rrt_params(problem, params)
    return run_native_continuous(
        problem.space,
        problem.start,
        problem.goal,
        resolved_params,
        resolve_rng(rng),
        planner="rrt_connect",
        trace=trace,
    )


__all__ = ["RrtConnect", "plan_rrt_connect"]
