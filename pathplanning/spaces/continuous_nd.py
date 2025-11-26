"""Shared geometric helpers for continuous N-dimensional spaces."""

from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence

import numpy as np

from pathplanning.core.types import Float, FloatArray


def as_state_nd(value: Sequence[Float] | FloatArray, name: str, dim: int) -> FloatArray:
    """Normalize one state vector and validate dimensionality."""
    state = np.asarray(value, dtype=float)
    if state.shape != (dim,):
        raise ValueError(f"{name} must be shape ({dim},), got {state.shape}")
    return state


def in_bounds_nd(
    point: Sequence[Float] | FloatArray,
    lower_bound: Sequence[Float] | FloatArray,
    upper_bound: Sequence[Float] | FloatArray,
    *,
    dim: int,
) -> bool:
    """Return ``True`` when ``point`` lies inside inclusive axis-aligned bounds."""
    state = as_state_nd(point, "point", dim)
    lower = as_state_nd(lower_bound, "lower_bound", dim)
    upper = as_state_nd(upper_bound, "upper_bound", dim)
    return bool(np.all(state >= lower) and np.all(state <= upper))


def euclidean_distance_nd(
    a: Sequence[Float] | FloatArray,
    b: Sequence[Float] | FloatArray,
    *,
    dim: int,
) -> float:
    """Compute Euclidean distance between two states."""
    start = as_state_nd(a, "a", dim)
    end = as_state_nd(b, "b", dim)
    return float(np.linalg.norm(end - start))


def steer_towards_nd(
    a: Sequence[Float] | FloatArray,
    b: Sequence[Float] | FloatArray,
    step: Float,
    *,
    dim: int,
) -> FloatArray:
    """Steer from ``a`` toward ``b`` by at most ``step``."""
    start = as_state_nd(a, "a", dim)
    target = as_state_nd(b, "b", dim)
    step_size = float(step)
    if step_size <= 0.0:
        raise ValueError("step_size must be > 0")

    direction = target - start
    distance = float(np.linalg.norm(direction))
    if distance <= step_size:
        return target
    return start + (direction / distance) * step_size


def sample_valid_state_nd(
    rng: np.random.Generator,
    lower_bound: Sequence[Float] | FloatArray,
    upper_bound: Sequence[Float] | FloatArray,
    is_state_valid: Callable[[FloatArray], bool],
    *,
    dim: int,
    max_tries: int = 1000,
    goal_state: Sequence[Float] | FloatArray | None = None,
    goal_sample_rate: float = 0.0,
) -> FloatArray:
    """Sample one state inside bounds and prefer valid states when possible.

    If no valid state is found within ``max_tries``, return the last sampled
    bounded state as a deterministic fallback.
    """

    if isinstance(max_tries, bool) or type(max_tries) is not int:
        raise TypeError("max_tries must be an integer")
    if max_tries <= 0:
        raise ValueError("max_tries must be > 0")

    goal_rate = float(goal_sample_rate)
    if goal_rate < 0.0 or goal_rate > 1.0:
        raise ValueError("goal_sample_rate must be in [0, 1]")

    lower = as_state_nd(lower_bound, "lower_bound", dim)
    upper = as_state_nd(upper_bound, "upper_bound", dim)

    if goal_state is not None and rng.random() < goal_rate:
        goal = as_state_nd(goal_state, "goal_state", dim)
        if is_state_valid(goal):
            return goal

    last_sample: FloatArray | None = None
    for _ in range(max_tries):
        candidate = np.asarray(rng.uniform(lower, upper), dtype=float)
        last_sample = candidate
        if is_state_valid(candidate):
            return candidate

    if last_sample is None:
        return np.asarray(rng.uniform(lower, upper), dtype=float)
    return last_sample


def backtrack_path_edges_nd(
    *,
    start: Sequence[Float] | FloatArray,
    goal: Sequence[Float] | FloatArray,
    parent_by_node: Mapping[tuple[float, ...], tuple[float, ...]],
    dim: int,
    distance_fn: Callable[[FloatArray, FloatArray], float] | None = None,
) -> tuple[list[FloatArray], float]:
    """Backtrack from ``goal`` to ``start`` using one parent map."""

    start_state = as_state_nd(start, "start", dim)
    goal_state = as_state_nd(goal, "goal", dim)
    start_node = tuple(float(value) for value in start_state)
    node = tuple(float(value) for value in goal_state)

    measure_distance: Callable[[FloatArray, FloatArray], float]
    if distance_fn is None:

        def _default_distance(a: FloatArray, b: FloatArray) -> float:
            return euclidean_distance_nd(a, b, dim=dim)

        measure_distance = _default_distance
    else:
        measure_distance = distance_fn

    path_edges: list[FloatArray] = []
    total_dist = 0.0
    max_hops = max(1, len(parent_by_node) + 1)
    hops = 0
    while node != start_node:
        if node not in parent_by_node:
            raise KeyError("parent_by_node does not contain a full path from goal to start")
        parent = parent_by_node[node]
        child_vec = np.asarray(node, dtype=float)
        parent_vec = np.asarray(parent, dtype=float)
        path_edges.append(np.asarray([child_vec, parent_vec], dtype=float))
        total_dist += float(measure_distance(child_vec, parent_vec))
        node = parent

        hops += 1
        if hops > max_hops:
            raise RuntimeError("Detected a cycle while backtracking path edges")

    return path_edges, total_dist


__all__ = [
    "as_state_nd",
    "in_bounds_nd",
    "euclidean_distance_nd",
    "steer_towards_nd",
    "sample_valid_state_nd",
    "backtrack_path_edges_nd",
]
