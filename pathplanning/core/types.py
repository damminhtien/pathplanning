"""Shared typing aliases for core planner modules."""

from __future__ import annotations

from typing import TypeAlias

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
