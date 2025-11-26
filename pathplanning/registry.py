"""Internal planner registry mapping planner ids to callable implementations."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any, Literal, Protocol

from pathplanning.core.contracts import ContinuousProblem, DiscreteProblem, State
from pathplanning.core.params import RrtParams
from pathplanning.core.results import PlanResult
from pathplanning.core.types import RNG
from pathplanning.planners.sampling.abit_star import plan_abit_star
from pathplanning.planners.sampling.bit_star import plan_bit_star
from pathplanning.planners.sampling.fmt_star import plan_fmt_star
from pathplanning.planners.sampling.informed_rrt_star import plan_informed_rrt_star
from pathplanning.planners.sampling.rrt import plan_rrt
from pathplanning.planners.sampling.rrt_connect import plan_rrt_connect
from pathplanning.planners.sampling.rrt_star import plan_rrt_star
from pathplanning.planners.search.anytime_astar import plan_anytime_astar
from pathplanning.planners.search.astar import plan_astar
from pathplanning.planners.search.bidirectional_astar import plan_bidirectional_astar
from pathplanning.planners.search.breadth_first_search import plan_breadth_first_search
from pathplanning.planners.search.depth_first_search import plan_depth_first_search
from pathplanning.planners.search.dijkstra import plan_dijkstra
from pathplanning.planners.search.greedy_best_first import plan_greedy_best_first
from pathplanning.planners.search.weighted_astar import plan_weighted_astar

ProblemKind = Literal["discrete", "continuous"]


class DiscretePlannerCallable(Protocol):
    """Callable contract for one discrete planner implementation."""

    def __call__(
        self,
        problem: DiscreteProblem[Any],
        *,
        params: Mapping[str, object] | None = None,
        rng: RNG | None = None,
    ) -> PlanResult: ...


class ContinuousPlannerCallable(Protocol):
    """Callable contract for one continuous planner implementation."""

    def __call__(
        self,
        problem: ContinuousProblem[State],
        *,
        params: RrtParams | Mapping[str, object] | None = None,
        rng: RNG | None = None,
    ) -> PlanResult: ...


SEARCH_PLANNERS: Mapping[str, DiscretePlannerCallable] = {
    "bfs": plan_breadth_first_search,
    "dfs": plan_depth_first_search,
    "greedy_best_first": plan_greedy_best_first,
    "astar": plan_astar,
    "bidirectional_astar": plan_bidirectional_astar,
    "dijkstra": plan_dijkstra,
    "weighted_astar": plan_weighted_astar,
    "anytime_astar": plan_anytime_astar,
}

SAMPLING_PLANNERS: Mapping[str, ContinuousPlannerCallable] = {
    "rrt": plan_rrt,
    "rrt_star": plan_rrt_star,
    "informed_rrt_star": plan_informed_rrt_star,
    "bit_star": plan_bit_star,
    "fmt_star": plan_fmt_star,
    "rrt_connect": plan_rrt_connect,
    "abit_star": plan_abit_star,
}


def list_planners(problem_kind: ProblemKind | None = None) -> list[str]:
    """Return registered planner names, optionally filtered by kind."""
    if problem_kind is None:
        return sorted([*SEARCH_PLANNERS, *SAMPLING_PLANNERS])
    if problem_kind == "discrete":
        return sorted(SEARCH_PLANNERS)
    return sorted(SAMPLING_PLANNERS)


def planner_modules(problem_kind: ProblemKind | None = None) -> list[str]:
    """Return unique Python modules that implement registered planners."""
    if problem_kind == "discrete":
        callables = SEARCH_PLANNERS.values()
    elif problem_kind == "continuous":
        callables = SAMPLING_PLANNERS.values()
    else:
        callables = [*SEARCH_PLANNERS.values(), *SAMPLING_PLANNERS.values()]
    return sorted({planner_fn.__module__ for planner_fn in callables})


def get_discrete_planner(planner: str) -> DiscretePlannerCallable:
    """Resolve one discrete planner by name."""
    try:
        return SEARCH_PLANNERS[planner]
    except KeyError as exc:
        raise KeyError(f"Unknown discrete planner: '{planner}'") from exc


def get_continuous_planner(planner: str) -> ContinuousPlannerCallable:
    """Resolve one continuous planner by name."""
    try:
        return SAMPLING_PLANNERS[planner]
    except KeyError as exc:
        raise KeyError(f"Unknown continuous planner: '{planner}'") from exc


__all__ = [
    "ProblemKind",
    "DiscretePlannerCallable",
    "ContinuousPlannerCallable",
    "SEARCH_PLANNERS",
    "SAMPLING_PLANNERS",
    "list_planners",
    "planner_modules",
    "get_discrete_planner",
    "get_continuous_planner",
]
