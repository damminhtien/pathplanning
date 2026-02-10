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
    AlgorithmSpec("search2d.breadth_first_search", "search", "2d", "Search_based_Planning.plan2d.breadth_first_search", SUPPORTED),
    AlgorithmSpec("search2d.depth_first_search", "search", "2d", "Search_based_Planning.plan2d.depth_first_search", SUPPORTED),
    AlgorithmSpec("search2d.best_first_search", "search", "2d", "Search_based_Planning.plan2d.best_first_search", SUPPORTED),
    AlgorithmSpec("search2d.dijkstra", "search", "2d", "Search_based_Planning.plan2d.dijkstra", SUPPORTED),
    AlgorithmSpec("search2d.astar", "search", "2d", "Search_based_Planning.plan2d.astar", SUPPORTED),
    AlgorithmSpec("search2d.bidirectional_astar", "search", "2d", "Search_based_Planning.plan2d.bidirectional_astar", SUPPORTED),
    AlgorithmSpec("search2d.anytime_dstar", "search", "2d", "Search_based_Planning.plan2d.anytime_dstar", SUPPORTED),
    AlgorithmSpec("search2d.anytime_repairing_astar", "search", "2d", "Search_based_Planning.plan2d.anytime_repairing_astar", SUPPORTED),
    AlgorithmSpec("search2d.lifelong_planning_astar", "search", "2d", "Search_based_Planning.plan2d.lifelong_planning_astar", SUPPORTED),
    AlgorithmSpec("search2d.learning_realtime_astar", "search", "2d", "Search_based_Planning.plan2d.learning_realtime_astar", SUPPORTED),
    AlgorithmSpec("search2d.realtime_adaptive_astar", "search", "2d", "Search_based_Planning.plan2d.realtime_adaptive_astar", SUPPORTED),
    AlgorithmSpec("search2d.dstar_lite", "search", "2d", "Search_based_Planning.plan2d.dstar_lite", SUPPORTED),
    AlgorithmSpec("search2d.dstar", "search", "2d", "Search_based_Planning.plan2d.dstar", SUPPORTED),
    # Search 3D
    AlgorithmSpec("search3d.astar", "search", "3d", "Search_based_Planning.Search_3D.Astar3D", SUPPORTED),
    AlgorithmSpec("search3d.bidirectional_astar", "search", "3d", "Search_based_Planning.Search_3D.bidirectional_Astar3D", SUPPORTED),
    AlgorithmSpec("search3d.lifelong_planning_astar", "search", "3d", "Search_based_Planning.Search_3D.LP_Astar3D", SUPPORTED),
    AlgorithmSpec("search3d.learning_realtime_astar", "search", "3d", "Search_based_Planning.Search_3D.LRT_Astar3D", SUPPORTED),
    AlgorithmSpec("search3d.realtime_adaptive_astar", "search", "3d", "Search_based_Planning.Search_3D.RTA_Astar3D", SUPPORTED),
    AlgorithmSpec("search3d.dstar", "search", "3d", "Search_based_Planning.Search_3D.Dstar3D", SUPPORTED),
    AlgorithmSpec("search3d.dstar_lite", "search", "3d", "Search_based_Planning.Search_3D.DstarLite3D", SUPPORTED),
    AlgorithmSpec("search3d.anytime_dstar", "search", "3d", "Search_based_Planning.Search_3D.Anytime_Dstar3D", SUPPORTED),
    # Sampling 2D
    AlgorithmSpec("sampling2d.rrt", "sampling", "2d", "Sampling_based_Planning.rrt_2D.rrt", SUPPORTED),
    AlgorithmSpec("sampling2d.rrt_connect", "sampling", "2d", "Sampling_based_Planning.rrt_2D.rrt_connect", SUPPORTED),
    AlgorithmSpec("sampling2d.extended_rrt", "sampling", "2d", "Sampling_based_Planning.rrt_2D.extended_rrt", SUPPORTED),
    AlgorithmSpec("sampling2d.dynamic_rrt", "sampling", "2d", "Sampling_based_Planning.rrt_2D.dynamic_rrt", SUPPORTED),
    AlgorithmSpec("sampling2d.rrt_star", "sampling", "2d", "Sampling_based_Planning.rrt_2D.rrt_star", SUPPORTED),
    AlgorithmSpec("sampling2d.rrt_sharp", "sampling", "2d", "Sampling_based_Planning.rrt_2D.rrt_sharp", SUPPORTED),
    AlgorithmSpec("sampling2d.informed_rrt_star", "sampling", "2d", "Sampling_based_Planning.rrt_2D.informed_rrt_star", SUPPORTED),
    AlgorithmSpec("sampling2d.rrt_star_smart", "sampling", "2d", "Sampling_based_Planning.rrt_2D.rrt_star_smart", SUPPORTED),
    AlgorithmSpec("sampling2d.dubins_rrt_star", "sampling", "2d", "Sampling_based_Planning.rrt_2D.dubins_rrt_star", SUPPORTED),
    AlgorithmSpec("sampling2d.fast_marching_trees", "sampling", "2d", "Sampling_based_Planning.rrt_2D.fast_marching_trees", SUPPORTED),
    AlgorithmSpec("sampling2d.batch_informed_trees", "sampling", "2d", "Sampling_based_Planning.rrt_2D.batch_informed_trees", SUPPORTED),
    AlgorithmSpec("sampling2d.advanced_batch_informed_trees", "sampling", "2d", "Sampling_based_Planning.rrt_2D.advanced_batch_informed_trees", SUPPORTED),
    AlgorithmSpec("sampling2d.adaptively_informed_trees", "sampling", "2d", "Sampling_based_Planning.rrt_2D.adaptively_informed_trees", SUPPORTED),
    # Sampling 3D
    AlgorithmSpec("sampling3d.rrt", "sampling", "3d", "Sampling_based_Planning.rrt_3D.rrt3D", SUPPORTED),
    AlgorithmSpec("sampling3d.rrt_connect", "sampling", "3d", "Sampling_based_Planning.rrt_3D.rrt_connect3D", SUPPORTED),
    AlgorithmSpec("sampling3d.extended_rrt", "sampling", "3d", "Sampling_based_Planning.rrt_3D.extend_rrt3D", SUPPORTED),
    AlgorithmSpec("sampling3d.dynamic_rrt", "sampling", "3d", "Sampling_based_Planning.rrt_3D.dynamic_rrt3D", SUPPORTED),
    AlgorithmSpec("sampling3d.rrt_star", "sampling", "3d", "Sampling_based_Planning.rrt_3D.rrt_star3D", SUPPORTED),
    AlgorithmSpec("sampling3d.informed_rrt_star", "sampling", "3d", "Sampling_based_Planning.rrt_3D.informed_rrt_star3D", SUPPORTED),
    AlgorithmSpec("sampling3d.rrt_star_smart", "sampling", "3d", "Sampling_based_Planning.rrt_3D.rrt_star_smart3D", SUPPORTED),
    AlgorithmSpec("sampling3d.fmt_star", "sampling", "3d", "Sampling_based_Planning.rrt_3D.FMT_star3D", SUPPORTED),
    AlgorithmSpec("sampling3d.bit_star", "sampling", "3d", "Sampling_based_Planning.rrt_3D.BIT_star3D", SUPPORTED),
    # Dropped from production surface.
    AlgorithmSpec(
        "sampling3d.abit_star",
        "sampling",
        "3d",
        "Sampling_based_Planning.rrt_3D.ABIT_star3D",
        DROPPED_INCOMPLETE,
        reason="Implementation is incomplete (stubbed methods and undefined symbols).",
    ),
]


_INDEX: Dict[str, AlgorithmSpec] = {spec.algorithm_id: spec for spec in _ALGORITHMS}


def list_algorithms() -> List[AlgorithmSpec]:
    """Return all algorithm metadata entries."""
    return list(_ALGORITHMS)


def list_supported_algorithms() -> List[AlgorithmSpec]:
    """Return production-supported algorithms."""
    return [spec for spec in _ALGORITHMS if spec.status == SUPPORTED]


def list_dropped_algorithms() -> List[AlgorithmSpec]:
    """Return dropped/incomplete algorithms excluded from production."""
    return [spec for spec in _ALGORITHMS if spec.status == DROPPED_INCOMPLETE]


def get_algorithm(algorithm_id: str) -> AlgorithmSpec:
    """Lookup one algorithm by id."""
    if algorithm_id not in _INDEX:
        raise KeyError(f"Unknown algorithm id: {algorithm_id}")
    return _INDEX[algorithm_id]
