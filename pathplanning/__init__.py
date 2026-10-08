"""PathPlanning reusable package API.

Keep root imports lightweight and side-effect free.
"""

from .api import Result, Stats, plan, plan_continuous, plan_discrete, plan_temporal
from .core.contracts import TemporalProblem
from .core.params import RrtParams
from .core.results import PlanResult, StopReason, TemporalPlanResult
from .core.trace import PlannerTrace, TraceOptions

__all__ = [
    "plan_discrete",
    "plan_temporal",
    "plan_continuous",
    "plan",
    "Result",
    "Stats",
    "RrtParams",
    "PlanResult",
    "TemporalPlanResult",
    "TemporalProblem",
    "StopReason",
    "PlannerTrace",
    "TraceOptions",
]
