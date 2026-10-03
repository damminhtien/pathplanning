"""Thin Python adapter for native C BIT* and ABIT* implementations."""

from __future__ import annotations

from collections.abc import Callable, Mapping
from typing import TypeAlias

import numpy as np

from pathplanning.core.contracts import ContinuousProblem, ContinuousSpace, State
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


class BatchInformedTreePlanner:
    """Object interface for native batch informed tree search."""

    planner_name = "bit_star"

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

    def plan(self, start: object, goal_region, *, trace: TraceOptions | None = None) -> PlanResult:
        return run_native_continuous(
            self.space,
            start,
            goal_region,
            self.params,
            self.rng,
            planner=self.planner_name,
            trace=trace,
        )


def _run(
    problem: ContinuousProblem[State], params, rng, planner: str, trace: TraceOptions | None
) -> PlanResult:
    validate_objective(problem.objective, "ABIT*" if planner == "abit_star" else "BIT*")
    resolved_params = coerce_rrt_params(problem, params)
    return run_native_continuous(
        problem.space,
        problem.start,
        problem.goal,
        resolved_params,
        resolve_rng(rng),
        planner=planner,
        trace=trace,
    )


def run_bit_star(
    problem: ContinuousProblem[State],
    *,
    params: RrtParams | Mapping[str, object] | None = None,
    rng: RNG | None = None,
    trace: TraceOptions | None = None,
) -> PlanResult:
    """Run native BIT* with batched samples and ordered implicit edges."""
    return _run(problem, params, rng, "bit_star", trace)


def run_abit_star(
    problem: ContinuousProblem[State],
    *,
    params: RrtParams | Mapping[str, object] | None = None,
    rng: RNG | None = None,
    trace: TraceOptions | None = None,
) -> PlanResult:
    """Run native ABIT* with decreasing inflation and truncation."""
    return _run(problem, params, rng, "abit_star", trace)


__all__ = ["BatchInformedTreePlanner", "IndexFactory", "run_bit_star", "run_abit_star"]
