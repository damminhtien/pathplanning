"""PathPlanning reusable package API.

Keep root imports lightweight and side-effect free.
"""

from .api import load_algorithm_module
from .registry import (
    SUPPORTED,
    AlgorithmSpec,
    get_algorithm,
    list_algorithms,
    list_supported_algorithms,
)
from .search2d import Heuristic, PlanConfig, Planner, PlanResult, Search2D

__all__ = [
    "SUPPORTED",
    "AlgorithmSpec",
    "get_algorithm",
    "list_algorithms",
    "list_supported_algorithms",
    "load_algorithm_module",
    "Search2D",
    "Planner",
    "Heuristic",
    "PlanConfig",
    "PlanResult",
]
