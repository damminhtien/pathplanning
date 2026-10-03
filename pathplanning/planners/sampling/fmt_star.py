"""Thin Python API for the native C FMT* planner."""

from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
from typing import TypeAlias

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

IndexFactory: TypeAlias = Callable[[int], NearestNeighborIndex]


class FmtStar:
    """Fixed-sample FMT* object wrapper over the native C implementation."""

    def __init__(
        self,
        space: ContinuousSpace[State],
        params: RrtParams,
        rng: np.random.Generator,
        nn_index_factory: IndexFactory | None = None,
    ) -> None:
        if nn_index_factory is not None:
            raise ValueError(
                "custom Python nearest-neighbor indexes are not supported by the C core"
            )
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
            self.space, start, goal_region, self.params, self.rng, planner="fmt_star", trace=trace
        )


def plan_fmt_star(
    problem: ContinuousProblem[State],
    *,
    params: RrtParams | Mapping[str, object] | None = None,
    rng: RNG | None = None,
    trace: TraceOptions | None = None,
) -> PlanResult:
    """Plan with native FMT* for an exact goal and additive path length."""
    validate_objective(problem.objective, "FMT*")
    resolved_params = coerce_rrt_params(problem, params)
    return run_native_continuous(
        problem.space,
        problem.start,
        problem.goal,
        resolved_params,
        resolve_rng(rng),
        planner="fmt_star",
        trace=trace,
    )


__all__ = ["FmtStar", "IndexFactory", "plan_fmt_star"]
