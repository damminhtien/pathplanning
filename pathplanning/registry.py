"""Production algorithm registry for PathPlanning."""

from __future__ import annotations

from dataclasses import dataclass

SUPPORTED = "supported"


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
    spec.algorithm_id for spec in _ALGORITHMS if not spec.entrypoint or not spec.entrypoint[0].isupper()
)
_INVALID_MODULE_PATHS = sorted(
    spec.algorithm_id for spec in _ALGORITHMS if not spec.module.startswith("pathplanning.planners.")
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
