"""Small validation helpers shared by Python sampling-planner interfaces."""

from __future__ import annotations

from collections.abc import Sequence

import numpy as np

from pathplanning.core.contracts import ContinuousSpace, GoalRegion, GoalState, State


def as_state(value: Sequence[float] | State, name: str, *, dim: int) -> State:
    """Normalize a coordinate-like value to a one-dimensional float state."""
    if dim <= 0:
        raise ValueError("state dimension must be > 0")
    state = np.asarray(value, dtype=float)
    if state.shape != (dim,):
        raise ValueError(f"{name} must be shape ({dim},), got {state.shape}")
    return state


def exact_goal_state(goal: GoalRegion[State], *, dim: int) -> State:
    """Return the exact point goal required by informed graph planners."""
    if not isinstance(goal, GoalState) or goal.radius != 0 or goal.distance_fn is not None:
        raise ValueError("this planner requires an exact GoalState with zero radius")
    return as_state(goal.state, "goal_state", dim=dim)


def validate_objective(objective: object | None, planner_name: str) -> None:
    """Reject objectives unsupported by additive path-length planners."""
    if objective is not None:
        raise ValueError(
            f"{planner_name} optimizes path length and does not accept a custom objective"
        )


def euclidean_distance(space: ContinuousSpace[State], start: State, end: State) -> float:
    """Validate Euclidean geometry required by informed planner bounds."""
    metric_distance = float(space.distance(start, end))
    coordinate_distance = float(np.linalg.norm(end - start))
    if not np.isfinite(metric_distance) or not np.isfinite(coordinate_distance):
        raise ValueError("space.distance must return a finite value")
    if metric_distance < 0.0:
        raise ValueError("space.distance must return a non-negative value")
    if not np.isclose(metric_distance, coordinate_distance, rtol=1e-9, atol=1e-12):
        raise ValueError("this planner requires Euclidean state-space distance")
    return metric_distance


__all__ = ["as_state", "euclidean_distance", "exact_goal_state", "validate_objective"]
