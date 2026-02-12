"""Production planner registry and dispatch metadata."""

from __future__ import annotations

from dataclasses import dataclass
import importlib
from types import ModuleType
from typing import Any, Callable, Literal, cast

from pathplanning.core.contracts import ContinuousProblem, DiscreteProblem
from pathplanning.core.params import RrtParams
from pathplanning.core.results import PlanResult
from pathplanning.core.types import RNG, S

SUPPORTED = "supported"


# ---------------------------------------------------------------------------
# Legacy algorithm-id registry (compatibility layer)
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class AlgorithmSpec:
    """Declarative metadata for one algorithm/module in production scope."""

    algorithm_id: str
    family: str
    dimension: str
    module: str
    entrypoint: str
    status: str
    reason: str | None = None


_ALGORITHMS: list[AlgorithmSpec] = [
    AlgorithmSpec(
        "sampling3d.rrt",
        "sampling",
        "3d",
        "pathplanning.planners.sampling.rrt",
        "RrtPlanner",
        SUPPORTED,
    ),
    AlgorithmSpec(
        "sampling3d.rrt_star",
        "sampling",
        "3d",
        "pathplanning.planners.sampling.rrt_star",
        "RrtStarPlanner",
        SUPPORTED,
    ),
]


_INDEX: dict[str, AlgorithmSpec] = {spec.algorithm_id: spec for spec in _ALGORITHMS}
_SUPPORTED_IDS = {spec.algorithm_id for spec in _ALGORITHMS if spec.status == SUPPORTED}
_INVALID_ENTRYPOINT_NAMES = sorted(
    spec.algorithm_id
    for spec in _ALGORITHMS
    if not spec.entrypoint or not spec.entrypoint[0].isupper()
)
_INVALID_MODULE_PATHS = sorted(
    spec.algorithm_id
    for spec in _ALGORITHMS
    if not spec.module.startswith("pathplanning.planners.")
)

if _INVALID_ENTRYPOINT_NAMES or _INVALID_MODULE_PATHS:
    raise RuntimeError(
        "Registry entrypoint contract mismatch. "
        f"invalid_entrypoint_names={_INVALID_ENTRYPOINT_NAMES}, "
        f"invalid_module_paths={_INVALID_MODULE_PATHS}"
    )


def list_algorithms() -> list[AlgorithmSpec]:
    """Return all algorithm metadata entries."""
    return list(_ALGORITHMS)


def list_supported_algorithms() -> list[AlgorithmSpec]:
    """Return production-supported algorithms with planner entrypoint contracts."""
    return [spec for spec in _ALGORITHMS if spec.status == SUPPORTED]


def expected_entrypoint_for_algorithm(algorithm_id: str) -> str:
    """Return the expected callable/class symbol name for one supported algorithm id."""
    spec = get_algorithm(algorithm_id)
    if spec.status != SUPPORTED:
        raise KeyError(f"No expected entrypoint contract defined for algorithm id: {algorithm_id}")
    return spec.entrypoint


def get_algorithm(algorithm_id: str) -> AlgorithmSpec:
    """Lookup one algorithm by id."""
    if algorithm_id not in _INDEX:
        raise KeyError(f"Unknown algorithm id: {algorithm_id}")
    return _INDEX[algorithm_id]


# ---------------------------------------------------------------------------
# Problem-oriented planner registry (new public API surface)
# ---------------------------------------------------------------------------
ProblemKind = Literal["discrete", "continuous"]

DiscretePlannerCallable = Callable[
    [DiscreteProblem[Any]],
    PlanResult,
]
ContinuousPlannerCallable = Callable[
    [ContinuousProblem[S]],
    PlanResult,
]
PlannerCallable = Callable[..., PlanResult]


@dataclass(frozen=True)
class PlannerSpec:
    """Planner metadata keyed by planner name and problem kind."""

    planner: str
    problem_kind: ProblemKind
    module: str
    entrypoint: str
    status: str = SUPPORTED
    reason: str | None = None


_PLANNERS: list[PlannerSpec] = [
    PlannerSpec(
        planner="astar",
        problem_kind="discrete",
        module="pathplanning.planners.search.entrypoints",
        entrypoint="plan_astar",
    ),
    PlannerSpec(
        planner="dijkstra",
        problem_kind="discrete",
        module="pathplanning.planners.search.entrypoints",
        entrypoint="plan_dijkstra",
    ),
    PlannerSpec(
        planner="rrt",
        problem_kind="continuous",
        module="pathplanning.planners.sampling.entrypoints",
        entrypoint="plan_rrt",
    ),
    PlannerSpec(
        planner="rrt_star",
        problem_kind="continuous",
        module="pathplanning.planners.sampling.entrypoints",
        entrypoint="plan_rrt_star",
    ),
    PlannerSpec(
        planner="informed_rrt_star",
        problem_kind="continuous",
        module="pathplanning.planners.sampling.entrypoints",
        entrypoint="plan_informed_rrt_star",
    ),
    PlannerSpec(
        planner="bit_star",
        problem_kind="continuous",
        module="pathplanning.planners.sampling.entrypoints",
        entrypoint="plan_bit_star",
    ),
    PlannerSpec(
        planner="fmt_star",
        problem_kind="continuous",
        module="pathplanning.planners.sampling.entrypoints",
        entrypoint="plan_fmt_star",
    ),
    PlannerSpec(
        planner="rrt_connect",
        problem_kind="continuous",
        module="pathplanning.planners.sampling.entrypoints",
        entrypoint="plan_rrt_connect",
    ),
    PlannerSpec(
        planner="abit_star",
        problem_kind="continuous",
        module="pathplanning.planners.sampling.entrypoints",
        entrypoint="plan_abit_star",
    ),
]


_PLANNER_INDEX: dict[tuple[ProblemKind, str], PlannerSpec] = {
    (spec.problem_kind, spec.planner): spec for spec in _PLANNERS
}

_ALGORITHM_TO_PLANNER: dict[str, tuple[ProblemKind, str]] = {
    "sampling3d.rrt": ("continuous", "rrt"),
    "sampling3d.rrt_star": ("continuous", "rrt_star"),
}


def list_planners(problem_kind: ProblemKind | None = None) -> list[PlannerSpec]:
    """Return registered planner metadata, optionally filtered by problem kind."""
    if problem_kind is None:
        return list(_PLANNERS)
    return [spec for spec in _PLANNERS if spec.problem_kind == problem_kind]


def list_supported_planners(problem_kind: ProblemKind | None = None) -> list[PlannerSpec]:
    """Return registered supported planners, optionally filtered by problem kind."""
    planners = list_planners(problem_kind)
    return [spec for spec in planners if spec.status == SUPPORTED]


def get_planner(planner: str, *, problem_kind: ProblemKind) -> PlannerSpec:
    """Lookup one planner by name and problem kind."""
    key = (problem_kind, planner)
    if key not in _PLANNER_INDEX:
        raise KeyError(f"Unknown planner '{planner}' for problem kind '{problem_kind}'")
    return _PLANNER_INDEX[key]


def planner_for_algorithm(algorithm_id: str) -> tuple[ProblemKind, str]:
    """Return problem kind and planner name mapped from legacy algorithm id."""
    if algorithm_id not in _ALGORITHM_TO_PLANNER:
        raise KeyError(f"Unknown algorithm id: {algorithm_id}")
    return _ALGORITHM_TO_PLANNER[algorithm_id]


def load_planner_module(planner: str, *, problem_kind: ProblemKind) -> ModuleType:
    """Load one planner module by planner name and problem kind."""
    spec = get_planner(planner, problem_kind=problem_kind)
    return importlib.import_module(spec.module)


def expected_entrypoint_for_planner(planner: str, *, problem_kind: ProblemKind) -> str:
    """Return expected entrypoint symbol for a planner name/problem kind pair."""
    return get_planner(planner, problem_kind=problem_kind).entrypoint


def resolve_planner_callable(planner: str, *, problem_kind: ProblemKind) -> PlannerCallable:
    """Resolve a registry planner name to a callable entrypoint."""
    module = load_planner_module(planner, problem_kind=problem_kind)
    entrypoint = expected_entrypoint_for_planner(planner, problem_kind=problem_kind)
    callable_entrypoint = getattr(module, entrypoint, None)
    if not callable(callable_entrypoint):
        raise TypeError(
            f"Registry entrypoint '{entrypoint}' for planner '{planner}' "
            f"({problem_kind}) is not callable"
        )
    return cast(PlannerCallable, callable_entrypoint)


def run_registered_discrete_planner(
    planner: str,
    problem: DiscreteProblem[Any],
    *,
    params: dict[str, object] | None = None,
    rng: RNG | None = None,
) -> PlanResult:
    """Run one registered discrete planner entrypoint."""
    planner_fn = resolve_planner_callable(planner, problem_kind="discrete")
    return planner_fn(problem, params=params, rng=rng)


def run_registered_continuous_planner(
    planner: str,
    problem: ContinuousProblem[S],
    *,
    params: RrtParams | dict[str, object] | None = None,
    rng: RNG | None = None,
) -> PlanResult:
    """Run one registered continuous planner entrypoint."""
    planner_fn = resolve_planner_callable(planner, problem_kind="continuous")
    return planner_fn(problem, params=params, rng=rng)


__all__ = [
    "SUPPORTED",
    "AlgorithmSpec",
    "PlannerSpec",
    "ProblemKind",
    "list_algorithms",
    "list_supported_algorithms",
    "expected_entrypoint_for_algorithm",
    "get_algorithm",
    "list_planners",
    "list_supported_planners",
    "get_planner",
    "planner_for_algorithm",
    "load_planner_module",
    "expected_entrypoint_for_planner",
    "resolve_planner_callable",
    "run_registered_discrete_planner",
    "run_registered_continuous_planner",
]
