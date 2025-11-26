"""Stable core contracts for planner implementations."""

from __future__ import annotations

from pathplanning.data_structures.tree_array import ArrayTree, Tree
from pathplanning.nn.index import (
    KDTreeIndex,
    KDTreeNnIndex,
    NaiveIndex,
    NaiveNnIndex,
    NearestNeighborIndex,
)

from .contracts import (
    ContinuousProblem,
    ContinuousSpace,
    ContinuousSpaceMetadata,
    DiscreteGraph,
    DiscreteProblem,
    DistanceAwareGoalRegion,
    ExactGoalTest,
    GoalRegion,
    GoalState,
    GoalTest,
    HeuristicDiscreteGraph,
    InterpolatingContinuousSpace,
    Objective,
    State,
    SupportsBatchMotionCheck,
    ValidatingDiscreteGraph,
)
from .params import RrtParams
from .results import PlanResult, StopReason
from .types import RNG, BoolArray, Float, FloatArray, Mat, N, NodeId, S, Vec

__all__ = [
    "DiscreteGraph",
    "HeuristicDiscreteGraph",
    "ValidatingDiscreteGraph",
    "GoalTest",
    "ExactGoalTest",
    "DiscreteProblem",
    "ContinuousSpace",
    "InterpolatingContinuousSpace",
    "ContinuousSpaceMetadata",
    "GoalRegion",
    "DistanceAwareGoalRegion",
    "GoalState",
    "SupportsBatchMotionCheck",
    "Objective",
    "ContinuousProblem",
    "PlanResult",
    "StopReason",
    "RrtParams",
    "State",
    "NearestNeighborIndex",
    "NaiveNnIndex",
    "KDTreeNnIndex",
    "NaiveIndex",
    "KDTreeIndex",
    "ArrayTree",
    "Tree",
    "NodeId",
    "Float",
    "FloatArray",
    "BoolArray",
    "S",
    "N",
    "RNG",
    "Vec",
    "Mat",
]
