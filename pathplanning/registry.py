"""Production algorithm registry for PathPlanning."""

from __future__ import annotations

from dataclasses import dataclass

SUPPORTED = "supported"
DROPPED_INCOMPLETE = "dropped_incomplete"


@dataclass(frozen=True)
class AlgorithmSpec:
    """Declarative metadata for one algorithm/module in production scope."""

    algorithm_id: str
    family: str
    dimension: str
    module: str
    status: str
    reason: str | None = None


_ALGORITHMS: list[AlgorithmSpec] = [
    AlgorithmSpec(
        "sampling3d.rrt",
        "sampling",
        "3d",
        "pathplanning.sampling_based_planning.rrt_3d.rrt",
        SUPPORTED,
    ),
    AlgorithmSpec(
        "sampling3d.rrt_star",
        "sampling",
        "3d",
        "pathplanning.sampling_based_planning.rrt_3d.rrt_star",
        SUPPORTED,
    ),
    AlgorithmSpec(
        "sampling2d.advanced_batch_informed_trees",
        "sampling",
        "2d",
        "pathplanning.sampling_based_planning.rrt_2d.advanced_batch_informed_trees",
        DROPPED_INCOMPLETE,
        "Not migrated to the production planner contract.",
    ),
    AlgorithmSpec(
        "sampling2d.adaptively_informed_trees",
        "sampling",
        "2d",
        "pathplanning.sampling_based_planning.rrt_2d.adaptively_informed_trees",
        DROPPED_INCOMPLETE,
        "Not migrated to the production planner contract.",
    ),
    AlgorithmSpec(
        "sampling2d.rrt_sharp",
        "sampling",
        "2d",
        "pathplanning.sampling_based_planning.rrt_2d.rrt_sharp",
        DROPPED_INCOMPLETE,
        "Not migrated to the production planner contract.",
    ),
    AlgorithmSpec(
        "sampling3d.abit_star",
        "sampling",
        "3d",
        "pathplanning.sampling_based_planning.rrt_3d.abit_star_3d",
        DROPPED_INCOMPLETE,
        "Not migrated to the production planner contract.",
    ),
    AlgorithmSpec(
        "sampling3d.rrt_star_smart",
        "sampling",
        "3d",
        "pathplanning.sampling_based_planning.rrt_3d.rrt_star_smart_3d",
        DROPPED_INCOMPLETE,
        "Not migrated to the production planner contract.",
    ),
]


_EXPECTED_ENTRYPOINTS: dict[str, str] = {
    "sampling3d.rrt": "RrtPlanner",
    "sampling3d.rrt_star": "RrtStarPlanner",
}

_INDEX: dict[str, AlgorithmSpec] = {spec.algorithm_id: spec for spec in _ALGORITHMS}
_SUPPORTED_IDS = {spec.algorithm_id for spec in _ALGORITHMS if spec.status == SUPPORTED}
_MISSING_ENTRYPOINTS = sorted(_SUPPORTED_IDS - set(_EXPECTED_ENTRYPOINTS))
_EXTRA_ENTRYPOINTS = sorted(set(_EXPECTED_ENTRYPOINTS) - _SUPPORTED_IDS)
_INVALID_ENTRYPOINT_NAMES = sorted(
    algorithm_id
    for algorithm_id, entrypoint in _EXPECTED_ENTRYPOINTS.items()
    if not entrypoint or not entrypoint[0].isupper()
)

if _MISSING_ENTRYPOINTS or _EXTRA_ENTRYPOINTS or _INVALID_ENTRYPOINT_NAMES:
    raise RuntimeError(
        "Registry entrypoint contract mismatch. "
        f"missing={_MISSING_ENTRYPOINTS}, extra={_EXTRA_ENTRYPOINTS}, "
        f"invalid_entrypoint_names={_INVALID_ENTRYPOINT_NAMES}"
    )


def list_algorithms() -> list[AlgorithmSpec]:
    """Return all algorithm metadata entries."""
    return list(_ALGORITHMS)


def list_supported_algorithms() -> list[AlgorithmSpec]:
    """Return production-supported algorithms with planner entrypoint contracts."""
    return [spec for spec in _ALGORITHMS if spec.status == SUPPORTED]


def list_dropped_algorithms() -> list[AlgorithmSpec]:
    """Return dropped/incomplete algorithms excluded from production."""
    return [spec for spec in _ALGORITHMS if spec.status == DROPPED_INCOMPLETE]


def expected_entrypoint_for_algorithm(algorithm_id: str) -> str:
    """Return the expected callable/class symbol name for one supported algorithm id."""
    if algorithm_id not in _EXPECTED_ENTRYPOINTS:
        raise KeyError(f"No expected entrypoint contract defined for algorithm id: {algorithm_id}")
    return _EXPECTED_ENTRYPOINTS[algorithm_id]


def get_algorithm(algorithm_id: str) -> AlgorithmSpec:
    """Lookup one algorithm by id."""
    if algorithm_id not in _INDEX:
        raise KeyError(f"Unknown algorithm id: {algorithm_id}")
    return _INDEX[algorithm_id]
