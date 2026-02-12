"""PathPlanning reusable package API.

Keep root imports lightweight and side-effect free.
"""

from .api import Result, Stats, load_algorithm_module, plan, run_planner
from .core.params import RrtParams
from .core.results import PlanResult, StopReason
from .registry import (
    SUPPORTED,
    AlgorithmSpec,
    get_algorithm,
    list_algorithms,
    list_supported_algorithms,
)

__all__ = [
    "SUPPORTED",
    "AlgorithmSpec",
    "get_algorithm",
    "list_algorithms",
    "list_supported_algorithms",
    "load_algorithm_module",
    "run_planner",
    "plan",
    "Result",
    "Stats",
    "RrtParams",
    "PlanResult",
    "StopReason",
]
