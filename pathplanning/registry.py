"""Production algorithm registry for PathPlanningV2."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional


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
    reason: Optional[str] = None


_ALGORITHMS: List[AlgorithmSpec] = [
    # Search 2D (production facade-backed)
    AlgorithmSpec(
        "search2d.breadth_first_search",
        "search",
        "2d",
        "pathplanning.search_based_planning.plan2d.breadth_first_search",
        SUPPORTED,
    ),
    AlgorithmSpec(
        "search2d.depth_first_search",
        "search",
        "2d",
        "pathplanning.search_based_planning.plan2d.depth_first_search",
        SUPPORTED,
    ),
    AlgorithmSpec(
        "search2d.best_first_search",
        "search",
        "2d",
        "pathplanning.search_based_planning.plan2d.best_first_search",
        SUPPORTED,
    ),
    AlgorithmSpec(
        "search2d.dijkstra",
        "search",
        "2d",
        "pathplanning.search_based_planning.plan2d.dijkstra",
        SUPPORTED,
    ),
    AlgorithmSpec(
        "search2d.astar",
        "search",
        "2d",
        "pathplanning.search_based_planning.plan2d.astar",
        SUPPORTED,
    ),
    AlgorithmSpec(
        "search2d.bidirectional_astar",
        "search",
        "2d",
        "pathplanning.search_based_planning.plan2d.bidirectional_astar",
        SUPPORTED,
    ),
    AlgorithmSpec(
        "search2d.anytime_dstar",
        "search",
        "2d",
        "pathplanning.search_based_planning.plan2d.anytime_dstar",
        SUPPORTED,
    ),
    AlgorithmSpec(
        "search2d.anytime_repairing_astar",
        "search",
        "2d",
        "pathplanning.search_based_planning.plan2d.anytime_repairing_astar",
        SUPPORTED,
    ),
    AlgorithmSpec(
        "search2d.lifelong_planning_astar",
        "search",
        "2d",
        "pathplanning.search_based_planning.plan2d.lifelong_planning_astar",
        SUPPORTED,
    ),
    AlgorithmSpec(
        "search2d.learning_realtime_astar",
        "search",
        "2d",
        "pathplanning.search_based_planning.plan2d.learning_realtime_astar",
        SUPPORTED,
    ),
    AlgorithmSpec(
        "search2d.realtime_adaptive_astar",
        "search",
        "2d",
        "pathplanning.search_based_planning.plan2d.realtime_adaptive_astar",
        SUPPORTED,
    ),
    AlgorithmSpec(
        "search2d.dstar_lite",
        "search",
        "2d",
        "pathplanning.search_based_planning.plan2d.dstar_lite",
        SUPPORTED,
    ),
    AlgorithmSpec(
        "search2d.dstar",
        "search",
        "2d",
        "pathplanning.search_based_planning.plan2d.dstar",
        SUPPORTED,
    ),
    # Search 3D
    AlgorithmSpec(
        "search3d.astar",
        "search",
        "3d",
        "pathplanning.search_based_planning.search_3d.astar_3d",
        SUPPORTED,
    ),
    AlgorithmSpec(
        "search3d.bidirectional_astar",
        "search",
        "3d",
        "pathplanning.search_based_planning.search_3d.bidirectional_astar_3d",
        SUPPORTED,
    ),
    AlgorithmSpec(
        "search3d.lifelong_planning_astar",
        "search",
        "3d",
        "pathplanning.search_based_planning.search_3d.lp_astar_3d",
        SUPPORTED,
    ),
    AlgorithmSpec(
        "search3d.learning_realtime_astar",
        "search",
        "3d",
        "pathplanning.search_based_planning.search_3d.lrt_astar_3d",
        SUPPORTED,
    ),
    AlgorithmSpec(
        "search3d.realtime_adaptive_astar",
        "search",
        "3d",
        "pathplanning.search_based_planning.search_3d.rta_astar_3d",
        SUPPORTED,
    ),
    AlgorithmSpec(
        "search3d.dstar",
        "search",
        "3d",
        "pathplanning.search_based_planning.search_3d.dstar_3d",
        SUPPORTED,
    ),
    AlgorithmSpec(
        "search3d.dstar_lite",
        "search",
        "3d",
        "pathplanning.search_based_planning.search_3d.dstar_lite_3d",
        SUPPORTED,
    ),
    AlgorithmSpec(
        "search3d.anytime_dstar",
        "search",
        "3d",
        "pathplanning.search_based_planning.search_3d.anytime_dstar_3d",
        SUPPORTED,
    ),
    # Sampling 2D
    AlgorithmSpec(
        "sampling2d.rrt",
        "sampling",
        "2d",
        "pathplanning.sampling_based_planning.rrt_2d.rrt",
        SUPPORTED,
    ),
    AlgorithmSpec(
        "sampling2d.rrt_connect",
        "sampling",
        "2d",
        "pathplanning.sampling_based_planning.rrt_2d.rrt_connect",
        SUPPORTED,
    ),
    AlgorithmSpec(
        "sampling2d.extended_rrt",
        "sampling",
        "2d",
        "pathplanning.sampling_based_planning.rrt_2d.extended_rrt",
        SUPPORTED,
    ),
    AlgorithmSpec(
        "sampling2d.dynamic_rrt",
        "sampling",
        "2d",
        "pathplanning.sampling_based_planning.rrt_2d.dynamic_rrt",
        SUPPORTED,
    ),
    AlgorithmSpec(
        "sampling2d.rrt_star",
        "sampling",
        "2d",
        "pathplanning.sampling_based_planning.rrt_2d.rrt_star",
        SUPPORTED,
    ),
    AlgorithmSpec(
        "sampling2d.rrt_sharp",
        "sampling",
        "2d",
        "pathplanning.sampling_based_planning.rrt_2d.rrt_sharp",
        DROPPED_INCOMPLETE,
        reason="Module is empty and does not expose a planner entry point.",
    ),
    AlgorithmSpec(
        "sampling2d.informed_rrt_star",
        "sampling",
        "2d",
        "pathplanning.sampling_based_planning.rrt_2d.informed_rrt_star",
        SUPPORTED,
    ),
    AlgorithmSpec(
        "sampling2d.rrt_star_smart",
        "sampling",
        "2d",
        "pathplanning.sampling_based_planning.rrt_2d.rrt_star_smart",
        SUPPORTED,
    ),
    AlgorithmSpec(
        "sampling2d.dubins_rrt_star",
        "sampling",
        "2d",
        "pathplanning.sampling_based_planning.rrt_2d.dubins_rrt_star",
        SUPPORTED,
    ),
    AlgorithmSpec(
        "sampling2d.fast_marching_trees",
        "sampling",
        "2d",
        "pathplanning.sampling_based_planning.rrt_2d.fast_marching_trees",
        SUPPORTED,
    ),
    AlgorithmSpec(
        "sampling2d.batch_informed_trees",
        "sampling",
        "2d",
        "pathplanning.sampling_based_planning.rrt_2d.batch_informed_trees",
        SUPPORTED,
    ),
    AlgorithmSpec(
        "sampling2d.advanced_batch_informed_trees",
        "sampling",
        "2d",
        "pathplanning.sampling_based_planning.rrt_2d.advanced_batch_informed_trees",
        DROPPED_INCOMPLETE,
        reason="Module is empty and does not expose a planner entry point.",
    ),
    AlgorithmSpec(
        "sampling2d.adaptively_informed_trees",
        "sampling",
        "2d",
        "pathplanning.sampling_based_planning.rrt_2d.adaptively_informed_trees",
        DROPPED_INCOMPLETE,
        reason="Module is empty and does not expose a planner entry point.",
    ),
    # Sampling 3D
    AlgorithmSpec(
        "sampling3d.rrt",
        "sampling",
        "3d",
        "pathplanning.sampling_based_planning.rrt_3d.rrt_3d",
        SUPPORTED,
    ),
    AlgorithmSpec(
        "sampling3d.rrt_connect",
        "sampling",
        "3d",
        "pathplanning.sampling_based_planning.rrt_3d.rrt_connect_3d",
        SUPPORTED,
    ),
    AlgorithmSpec(
        "sampling3d.extended_rrt",
        "sampling",
        "3d",
        "pathplanning.sampling_based_planning.rrt_3d.extend_rrt_3d",
        SUPPORTED,
    ),
    AlgorithmSpec(
        "sampling3d.dynamic_rrt",
        "sampling",
        "3d",
        "pathplanning.sampling_based_planning.rrt_3d.dynamic_rrt_3d",
        SUPPORTED,
    ),
    AlgorithmSpec(
        "sampling3d.rrt_star",
        "sampling",
        "3d",
        "pathplanning.sampling_based_planning.rrt_3d.rrt_star_3d",
        SUPPORTED,
    ),
    AlgorithmSpec(
        "sampling3d.informed_rrt_star",
        "sampling",
        "3d",
        "pathplanning.sampling_based_planning.rrt_3d.informed_rrt_star_3d",
        SUPPORTED,
    ),
    AlgorithmSpec(
        "sampling3d.rrt_star_smart",
        "sampling",
        "3d",
        "pathplanning.sampling_based_planning.rrt_3d.rrt_star_smart_3d",
        DROPPED_INCOMPLETE,
        reason="Module is empty and does not expose a planner entry point.",
    ),
    AlgorithmSpec(
        "sampling3d.fmt_star",
        "sampling",
        "3d",
        "pathplanning.sampling_based_planning.rrt_3d.fmt_star_3d",
        SUPPORTED,
    ),
    AlgorithmSpec(
        "sampling3d.bit_star",
        "sampling",
        "3d",
        "pathplanning.sampling_based_planning.rrt_3d.bit_star_3d",
        SUPPORTED,
    ),
    # Dropped from production surface.
    AlgorithmSpec(
        "sampling3d.abit_star",
        "sampling",
        "3d",
        "pathplanning.sampling_based_planning.rrt_3d.abit_star_3d",
        DROPPED_INCOMPLETE,
        reason="Implementation is incomplete (stubbed methods and undefined symbols).",
    ),
]


_EXPECTED_ENTRYPOINTS: Dict[str, str] = {
    "sampling2d.batch_informed_trees": "BITStar",
    "sampling2d.dubins_rrt_star": "DubinsRRTStar",
    "sampling2d.dynamic_rrt": "DynamicRrt",
    "sampling2d.extended_rrt": "ExtendedRrt",
    "sampling2d.fast_marching_trees": "FMT",
    "sampling2d.informed_rrt_star": "IRrtStar",
    "sampling2d.rrt": "Rrt",
    "sampling2d.rrt_connect": "RrtConnect",
    "sampling2d.rrt_star": "RrtStar",
    "sampling2d.rrt_star_smart": "RrtStarSmart",
    "sampling3d.bit_star": "BIT_star",
    "sampling3d.dynamic_rrt": "DynamicRRT3D",
    "sampling3d.extended_rrt": "extend_rrt",
    "sampling3d.fmt_star": "FMT_star",
    "sampling3d.informed_rrt_star": "IRRT",
    "sampling3d.rrt": "rrt",
    "sampling3d.rrt_connect": "rrt_connect",
    "sampling3d.rrt_star": "rrtstar",
    "search2d.anytime_dstar": "AnytimeDstar",
    "search2d.anytime_repairing_astar": "AnytimeRepairingAstar",
    "search2d.astar": "Astar",
    "search2d.best_first_search": "BestFirstSearch",
    "search2d.bidirectional_astar": "BidirectionalAstar",
    "search2d.breadth_first_search": "BreadthFirstSearch",
    "search2d.depth_first_search": "DepthFirstSearch",
    "search2d.dijkstra": "Dijkstra",
    "search2d.dstar": "Dstar",
    "search2d.dstar_lite": "DStarLite",
    "search2d.learning_realtime_astar": "LearningRealtimeAstar",
    "search2d.lifelong_planning_astar": "LifelongPlanningAstar",
    "search2d.realtime_adaptive_astar": "RealtimeAdaptiveAstar",
    "search3d.anytime_dstar": "Anytime_Dstar",
    "search3d.astar": "Weighted_A_star",
    "search3d.bidirectional_astar": "Weighted_A_star",
    "search3d.dstar": "D_star",
    "search3d.dstar_lite": "D_star_Lite",
    "search3d.learning_realtime_astar": "LRT_A_star2",
    "search3d.lifelong_planning_astar": "Lifelong_Astar",
    "search3d.realtime_adaptive_astar": "RTA_A_star",
}

_INDEX: Dict[str, AlgorithmSpec] = {spec.algorithm_id: spec for spec in _ALGORITHMS}
_SUPPORTED_IDS = {spec.algorithm_id for spec in _ALGORITHMS if spec.status == SUPPORTED}
_MISSING_ENTRYPOINTS = sorted(_SUPPORTED_IDS - set(_EXPECTED_ENTRYPOINTS))
_EXTRA_ENTRYPOINTS = sorted(set(_EXPECTED_ENTRYPOINTS) - _SUPPORTED_IDS)

if _MISSING_ENTRYPOINTS or _EXTRA_ENTRYPOINTS:
    raise RuntimeError(
        "Registry entrypoint contract mismatch. "
        f"missing={_MISSING_ENTRYPOINTS}, extra={_EXTRA_ENTRYPOINTS}"
    )


def list_algorithms() -> List[AlgorithmSpec]:
    """Return all algorithm metadata entries."""
    return list(_ALGORITHMS)


def list_supported_algorithms() -> List[AlgorithmSpec]:
    """Return production-supported algorithms."""
    return [spec for spec in _ALGORITHMS if spec.status == SUPPORTED]


def list_dropped_algorithms() -> List[AlgorithmSpec]:
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
