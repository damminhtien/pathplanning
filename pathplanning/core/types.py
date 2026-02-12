"""Shared typing aliases for core planner modules."""

from __future__ import annotations

from typing import TypeAlias, TypeVar

import numpy as np
from numpy.typing import NDArray

NodeId: TypeAlias = int
Float: TypeAlias = float

FloatArray: TypeAlias = NDArray[np.float64]
BoolArray: TypeAlias = NDArray[np.bool_]

# Shape intentions:
# - Vec: (dim,)
# - Mat: (n, dim)
Vec: TypeAlias = FloatArray
Mat: TypeAlias = FloatArray

# Generic state/node variables for planner contracts.
S = TypeVar("S")  # Continuous state type
N = TypeVar("N")  # Discrete graph node type

# RNG policy:
# - Core contracts accept an explicit RNG value.
# - Callers/planners should pass a local ``np.random.Generator`` instance.
# - Avoid module-level ``np.random.*`` helpers for planner randomness.
RNG: TypeAlias = np.random.Generator
