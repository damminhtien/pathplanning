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
from .core.contracts import MultiAgentProblem, StateLatticePrimitive, TemporalProblem
from .core.params import (
    HybridAStarParams,
    JitParams,
    RitParams,
    RoadmapParams,
    RrtParams,
    StateLatticeParams,
)
from .core.results import (
    KinematicPlanResult,
    MultiAgentPlanResult,
    PlanResult,
    StopReason,
    TemporalPlanResult,
)
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
    "RitParams",
    "JitParams",
    "HybridAStarParams",
    "StateLatticeParams",
    "StateLatticePrimitive",
    "RoadmapParams",
    "PlanResult",
    "TemporalPlanResult",
    "MultiAgentPlanResult",
    "KinematicPlanResult",
    "TemporalProblem",
    "MultiAgentProblem",
    "StopReason",
    "PlannerTrace",
    "TraceOptions",
]
