"""Internal planner registry mapping planner ids to callable implementations."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from types import MappingProxyType
from typing import Any, Literal, Protocol, Union, cast

from pathplanning.core.contracts import ContinuousProblem, DiscreteProblem, State
from pathplanning.core.params import RrtParams
from pathplanning.core.results import PlanResult
from pathplanning.core.trace import TraceOptions
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
from pathplanning.planners.search.bidirectional_dijkstra import plan_bidirectional_dijkstra
from pathplanning.planners.search.breadth_first_search import plan_breadth_first_search
from pathplanning.planners.search.depth_first_search import plan_depth_first_search
from pathplanning.planners.search.dijkstra import plan_dijkstra
from pathplanning.planners.search.greedy_best_first import plan_greedy_best_first
from pathplanning.planners.search.jump_point import plan_jps
from pathplanning.planners.search.jump_point_weighted import plan_jpsw
from pathplanning.planners.search.reexp_astar import plan_reexp_astar
from pathplanning.planners.search.theta_star import plan_theta_star
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
        trace: TraceOptions | None = None,
    ) -> PlanResult: ...


class ContinuousPlannerCallable(Protocol):
    """Callable contract for one continuous planner implementation."""

    def __call__(
        self,
        problem: ContinuousProblem[State],
        *,
        params: RrtParams | Mapping[str, object] | None = None,
        rng: RNG | None = None,
        trace: TraceOptions | None = None,
    ) -> PlanResult: ...


PlannerCallable = Union[DiscretePlannerCallable, ContinuousPlannerCallable]


@dataclass(frozen=True, slots=True)
class PlannerSpec:
    """Metadata and implementation for one registered production planner."""

    problem_kind: ProblemKind
    planner: PlannerCallable
    constraints: tuple[str, ...] = ()


_OPTIMAL_SAMPLING_CONSTRAINTS = (
    "require exact point goals, Euclidean state-space distance, and additive path length; "
    "custom objectives are rejected because the search bounds rely on additive path length.",
)

# Keep every production planner declaration here. Public listings, dispatch, tests,
# and the supported-planner document derive from this registry.
_PLANNER_SPECS: tuple[tuple[str, PlannerSpec], ...] = (
    ("bfs", PlannerSpec("discrete", plan_breadth_first_search)),
    ("dfs", PlannerSpec("discrete", plan_depth_first_search)),
    ("greedy_best_first", PlannerSpec("discrete", plan_greedy_best_first)),
    ("astar", PlannerSpec("discrete", plan_astar)),
    (
        "bidirectional_dijkstra",
        PlannerSpec(
            "discrete",
            plan_bidirectional_dijkstra,
            (
                "uses two Dijkstra frontiers and does not evaluate a heuristic; it requires an exact goal node.",
            ),
        ),
    ),
    (
        "bidirectional_astar",
        PlannerSpec(
            "discrete",
            plan_bidirectional_astar,
            (
                "runs the same native kernel over potential-reweighted edges and requires a consistent "
                "heuristic and an exact goal node.",
            ),
        ),
    ),
    ("dijkstra", PlannerSpec("discrete", plan_dijkstra)),
    ("weighted_astar", PlannerSpec("discrete", plan_weighted_astar)),
    (
        "reexp_astar",
        PlannerSpec(
            "discrete",
            plan_reexp_astar,
            (
                "Weighted A* with conditional closed-node re-expansion; "
                "r uses r_mode (abs, rel_edge, or rel_g), and "
                "tie_break is g_low or g_high.",
            ),
        ),
    ),
    ("anytime_astar", PlannerSpec("discrete", plan_anytime_astar)),
    (
        "jps",
        PlannerSpec(
            "discrete",
            plan_jps,
            (
                "requires uniform Grid2DSearchSpace costs, standard 8-connected motions, "
                "and prohibits diagonal corner cutting.",
            ),
        ),
    ),
    (
        "jpsw",
        PlannerSpec(
            "discrete",
            plan_jpsw,
            (
                "requires TerrainCostGrid2D with positive finite terrain costs, standard 8-connected "
                "motions, exact cell goals, and no diagonal corner cutting.",
            ),
        ),
    ),
    (
        "theta_star",
        PlannerSpec(
            "discrete",
            plan_theta_star,
            (
                "requires standard 8-connected Grid2DSearchSpace or TerrainCostGrid2D; "
                "line of sight rejects blocked cells and corner cutting.",
            ),
        ),
    ),
    ("rrt", PlannerSpec("continuous", plan_rrt)),
    ("rrt_star", PlannerSpec("continuous", plan_rrt_star)),
    (
        "informed_rrt_star",
        PlannerSpec("continuous", plan_informed_rrt_star, _OPTIMAL_SAMPLING_CONSTRAINTS),
    ),
    (
        "fmt_star",
        PlannerSpec(
            "continuous",
            plan_fmt_star,
            (*_OPTIMAL_SAMPLING_CONSTRAINTS, "uses `sample_count` as its fixed sample set size."),
        ),
    ),
    (
        "bit_star",
        PlannerSpec(
            "continuous",
            plan_bit_star,
            (
                *_OPTIMAL_SAMPLING_CONSTRAINTS,
                "use `sample_count` across batches and `batch_size` per batch.",
            ),
        ),
    ),
    (
        "abit_star",
        PlannerSpec(
            "continuous",
            plan_abit_star,
            (
                *_OPTIMAL_SAMPLING_CONSTRAINTS,
                "use `sample_count` across batches and `batch_size` per batch.",
            ),
        ),
    ),
    ("rrt_connect", PlannerSpec("continuous", plan_rrt_connect)),
)


def _build_registry(entries: tuple[tuple[str, PlannerSpec], ...]) -> Mapping[str, PlannerSpec]:
    registry: dict[str, PlannerSpec] = {}
    for name, spec in entries:
        if name in registry:
            raise ValueError(f"Duplicate planner name: {name}")
        registry[name] = spec
    return MappingProxyType(registry)


PLANNER_REGISTRY = _build_registry(_PLANNER_SPECS)


def list_planners(problem_kind: ProblemKind | None = None) -> list[str]:
    """Return registered planner names, optionally filtered by kind."""
    return sorted(
        name
        for name, spec in PLANNER_REGISTRY.items()
        if problem_kind is None or spec.problem_kind == problem_kind
    )


def planner_modules(problem_kind: ProblemKind | None = None) -> list[str]:
    """Return unique Python modules that implement registered planners."""
    return sorted(
        {
            getattr(spec.planner, "__module__")
            for spec in PLANNER_REGISTRY.values()
            if problem_kind is None or spec.problem_kind == problem_kind
        }
    )


def get_discrete_planner(planner: str) -> DiscretePlannerCallable:
    """Resolve one discrete planner by name."""
    spec = PLANNER_REGISTRY.get(planner)
    if spec is None or spec.problem_kind != "discrete":
        raise KeyError(f"Unknown discrete planner: '{planner}'")
    return cast(DiscretePlannerCallable, spec.planner)


def get_continuous_planner(planner: str) -> ContinuousPlannerCallable:
    """Resolve one continuous planner by name."""
    spec = PLANNER_REGISTRY.get(planner)
    if spec is None or spec.problem_kind != "continuous":
        raise KeyError(f"Unknown continuous planner: '{planner}'")
    return cast(ContinuousPlannerCallable, spec.planner)


__all__ = [
    "ProblemKind",
    "DiscretePlannerCallable",
    "ContinuousPlannerCallable",
    "PlannerCallable",
    "PlannerSpec",
    "PLANNER_REGISTRY",
    "list_planners",
    "planner_modules",
    "get_discrete_planner",
    "get_continuous_planner",
]
