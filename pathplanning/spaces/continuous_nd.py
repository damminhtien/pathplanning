"""Shared geometric helpers for continuous N-dimensional spaces."""

from __future__ import annotations

from collections.abc import Sequence

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


__all__ = [
    "as_state_nd",
    "in_bounds_nd",
    "euclidean_distance_nd",
    "steer_towards_nd",
]
