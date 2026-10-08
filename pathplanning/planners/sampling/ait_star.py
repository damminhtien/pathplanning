"""Adaptive informed tree search over a fixed random geometric graph."""

from __future__ import annotations

from collections.abc import Mapping

from pathplanning.core.contracts import ContinuousProblem, State
from pathplanning.core.params import RrtParams
from pathplanning.core.results import PlanResult
from pathplanning.core.trace import TraceOptions
from pathplanning.core.types import RNG
from pathplanning.native.continuous import run_native_continuous
from pathplanning.planners.sampling._internal.continuous import (
    euclidean_distance,
    exact_goal_state,
    validate_objective,
)
from pathplanning.planners.sampling._internal.problem_adapter import (
    coerce_rrt_params,
    resolve_rng,
)


def plan_ait_star(
    problem: ContinuousProblem[State],
    *,
    params: RrtParams | Mapping[str, object] | None = None,
    rng: RNG | None = None,
    trace: TraceOptions | None = None,
) -> PlanResult:
    """Plan with native AIT* for additive Euclidean path length."""
    validate_objective(problem.objective, "AIT*")
    resolved_params = coerce_rrt_params(problem, params)
    goal = exact_goal_state(problem.goal, dim=len(problem.start))
    euclidean_distance(problem.space, problem.start, goal)
    return run_native_continuous(
        problem.space,
        problem.start,
        problem.goal,
        resolved_params,
        resolve_rng(rng),
        planner="ait_star",
        trace=trace,
    )


__all__ = ["plan_ait_star"]
