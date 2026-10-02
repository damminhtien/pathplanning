"""Shared validation and sampling helpers for continuous planners."""

from __future__ import annotations

from collections.abc import Callable, Sequence
from typing import cast

import numpy as np

from pathplanning.core.contracts import (
    ContinuousSpace,
    GoalRegion,
    GoalState,
    State,
    SupportsCollisionStepMotionCheck,
)
from pathplanning.core.params import RrtParams
from pathplanning.core.types import RNG
from pathplanning.nn.index import (
    KDTreeNnIndex,
    NaiveNnIndex,
    NearestNeighborIndex,
)


def as_state(value: Sequence[float] | State, name: str, *, dim: int) -> State:
    """Normalize a coordinate-like value to a one-dimensional float state."""
    if dim <= 0:
        raise ValueError("state dimension must be > 0")
    state = np.asarray(value, dtype=float)
    if state.shape != (dim,):
        raise ValueError(f"{name} must be shape ({dim},), got {state.shape}")
    return state


def exact_goal_state(goal: GoalRegion[State], *, dim: int) -> State:
    """Return the exact point goal required by graph-based sampling planners."""
    if not isinstance(goal, GoalState) or goal.radius != 0 or goal.distance_fn is not None:
        raise ValueError("this planner requires an exact GoalState with zero radius")
    return as_state(goal.state, "goal_state", dim=dim)


def validate_objective(objective: object | None, planner_name: str) -> None:
    if objective is not None:
        raise ValueError(
            f"{planner_name} optimizes path length and does not accept a custom objective"
        )


def euclidean_distance(space: ContinuousSpace[State], start: State, end: State) -> float:
    """Return path length while enforcing Euclidean state-space geometry."""
    metric_distance = float(space.distance(start, end))
    coordinate_distance = float(np.linalg.norm(end - start))
    if not np.isfinite(metric_distance) or not np.isfinite(coordinate_distance):
        raise ValueError("space.distance must return a finite value")
    if metric_distance < 0.0:
        raise ValueError("space.distance must return a non-negative value")
    if not np.isclose(metric_distance, coordinate_distance, rtol=1e-9, atol=1e-12):
        raise ValueError("this planner requires Euclidean state-space distance")
    return metric_distance


def collect_free_samples(
    space: ContinuousSpace[State],
    rng: RNG,
    params: RrtParams,
    *,
    count: int,
    dim: int,
) -> np.ndarray:
    """Collect a bounded number of valid samples from the space contract."""
    samples = np.empty((count, dim), dtype=float)
    for sample_index in range(count):
        for _ in range(params.max_sample_tries):
            state = as_state(space.sample_free(rng), "sampled_state", dim=dim)
            if space.is_state_valid(state):
                samples[sample_index] = state
                break
        else:
            raise ValueError("could not sample the requested number of valid states")
    return samples


def build_nn_index(
    points: np.ndarray,
    *,
    factory: Callable[[int], NearestNeighborIndex] | None = None,
) -> NearestNeighborIndex:
    """Build one static nearest-neighbor index, with optional SciPy acceleration."""
    dim = int(points.shape[1])
    if factory is not None:
        index = factory(dim)
        index.build(points)
        return index
    try:
        index = KDTreeNnIndex(dim)
        index.build(points)
        return index
    except RuntimeError:
        index = NaiveNnIndex(dim)
        index.build(points)
        return index


def connection_radius(params: RrtParams, *, count: int, dim: int) -> float:
    """Compute a dimension-aware random-geometric-graph connection radius."""
    if count <= 1:
        return params.step_size
    radius = params.rrt_star_radius_gamma * (np.log(count) / count) ** (1.0 / dim)
    return float(min(params.step_size * params.rrt_star_radius_max_factor, radius))


def path_from_parents(states: np.ndarray, parents: np.ndarray, node: int) -> np.ndarray:
    path: list[np.ndarray] = []
    seen = 0
    while node >= 0:
        path.append(states[node])
        node = int(parents[node])
        seen += 1
        if seen > states.shape[0]:
            raise RuntimeError("planner parent chain contains a cycle")
    path.reverse()
    return np.asarray(path, dtype=float)


def motion_is_valid(
    space: ContinuousSpace[State],
    params: RrtParams,
    start: State,
    end: State,
) -> bool:
    """Use the planner-local collision step when the space supports it."""
    if hasattr(space, "is_motion_valid_with_step"):
        checker = cast(SupportsCollisionStepMotionCheck[State], space)
        return bool(checker.is_motion_valid_with_step(start, end, params.collision_step))
    return bool(space.is_motion_valid(start, end))


def sample_informed(
    space: ContinuousSpace[State],
    rng: RNG,
    params: RrtParams,
    start: State,
    goal: State,
    best_cost: float,
) -> State:
    """Draw from the prolate hyperspheroid with fallback to free-space sampling."""
    dim = int(start.size)
    delta = goal - start
    c_min = float(np.linalg.norm(delta))
    if not np.isfinite(best_cost) or best_cost <= c_min * (1.0 + 1e-12):
        return as_state(space.sample_free(rng), "sampled_state", dim=dim)

    major = best_cost * 0.5
    minor = 0.5 * np.sqrt(max(best_cost * best_cost - c_min * c_min, 0.0))
    center = 0.5 * (start + goal)
    rotation = np.eye(dim, dtype=float)
    if dim > 1 and c_min > 0.0:
        direction = delta / c_min
        first_axis = np.zeros(dim, dtype=float)
        first_axis[0] = 1.0
        reflection = first_axis - direction
        norm_sq = float(reflection @ reflection)
        if norm_sq > 1e-24:
            rotation -= (2.0 / norm_sq) * np.outer(reflection, reflection)

    for _ in range(params.max_sample_tries):
        direction = rng.normal(size=dim)
        norm = float(np.linalg.norm(direction))
        if norm <= 1e-15:
            continue
        unit_ball = direction * (float(rng.random()) ** (1.0 / dim) / norm)
        scaled = unit_ball.copy()
        scaled[0] *= major
        if dim > 1:
            scaled[1:] *= minor
        state = center + rotation @ scaled
        if space.is_state_valid(state):
            return state
    return as_state(space.sample_free(rng), "sampled_state", dim=dim)


__all__ = [
    "as_state",
    "build_nn_index",
    "collect_free_samples",
    "connection_radius",
    "euclidean_distance",
    "exact_goal_state",
    "motion_is_valid",
    "path_from_parents",
    "sample_informed",
    "validate_objective",
]
