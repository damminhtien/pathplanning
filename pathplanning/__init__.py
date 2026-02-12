"""PathPlanning reusable package API.

Keep root imports lightweight and side-effect free.
"""

from .api import (
    Result,
    Stats,
    load_algorithm_module,
    plan,
    plan_continuous,
    plan_discrete,
    run_planner,
)
from .core.params import RrtParams
from .core.results import PlanResult, StopReason
from .registry import (
    SUPPORTED,
    AlgorithmSpec,
    PlannerSpec,
    get_algorithm,
    get_planner,
    list_algorithms,
    list_planners,
    list_supported_planners,
    list_supported_algorithms,
)

__all__ = [
    "SUPPORTED",
    "AlgorithmSpec",
    "PlannerSpec",
    "get_algorithm",
    "get_planner",
    "list_algorithms",
    "list_planners",
    "list_supported_planners",
    "list_supported_algorithms",
    "load_algorithm_module",
    "run_planner",
    "plan_discrete",
    "plan_continuous",
    "plan",
    "Result",
    "Stats",
    "RrtParams",
    "PlanResult",
    "StopReason",
]
