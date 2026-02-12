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
    BatchConfigurationSpace,
    ConfigurationSpace,
    ContinuousProblem,
    ContinuousSpace,
    ContinuousSpaceMetadata,
    DistanceAwareGoalRegion,
    DiscreteGraph,
    DiscreteProblem,
    ExactGoalTest,
    GoalRegion,
    GoalState,
    GoalTest,
    HeuristicDiscreteGraph,
    InterpolatingContinuousSpace,
    Objective,
    SupportsBatchMotionCheck,
    State,
    ValidatingDiscreteGraph,
)
from .params import RrtParams
from .results import PlanResult, StopReason
from .types import BoolArray, Float, FloatArray, Mat, N, NodeId, RNG, S, Vec

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
    "ConfigurationSpace",
    "BatchConfigurationSpace",
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
