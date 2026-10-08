"""PathPlanning reusable package API.

Keep root imports lightweight and side-effect free.
"""

from .api import (
    Result,
    Stats,
    plan,
    plan_continuous,
    plan_discrete,
    plan_multi_agent,
    plan_temporal,
)
from .core.contracts import MultiAgentProblem, TemporalProblem
from .core.params import RoadmapParams, RrtParams
from .core.results import MultiAgentPlanResult, PlanResult, StopReason, TemporalPlanResult
from .core.trace import PlannerTrace, TraceOptions

__all__ = [
    "plan_discrete",
    "plan_temporal",
    "plan_multi_agent",
    "plan_continuous",
    "plan",
    "Result",
    "Stats",
    "RrtParams",
    "RoadmapParams",
    "PlanResult",
    "TemporalPlanResult",
    "MultiAgentPlanResult",
    "TemporalProblem",
    "MultiAgentProblem",
    "StopReason",
    "PlannerTrace",
    "TraceOptions",
]
