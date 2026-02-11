"""Shared typing aliases for production planner code."""

from __future__ import annotations

from typing import NewType, TypeAlias

import numpy as np
from numpy.typing import NDArray

FloatArray: TypeAlias = NDArray[np.float64]
IntArray: TypeAlias = NDArray[np.int64]
BoolArray: TypeAlias = NDArray[np.bool_]
State: TypeAlias = FloatArray
NodeId = NewType("NodeId", int)

__all__ = ["BoolArray", "FloatArray", "IntArray", "NodeId", "State"]
