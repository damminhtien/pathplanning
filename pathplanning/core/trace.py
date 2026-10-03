"""Optional diagnostic events captured by a separate native build."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
from typing import Any, Literal

import numpy as np
from numpy.typing import NDArray

TRACE_EVENT_DTYPE = np.dtype(
    [
        ("node", np.uint64),
        ("parent", np.uint64),
        ("value", np.float64),
        ("kind", np.uint32),
        ("side", np.uint32),
    ],
    align=True,
)


@dataclass(frozen=True, slots=True)
class TraceOptions:
    """Bound the native memory spent recording one planning run."""

    max_bytes: int = 32 * 1024 * 1024

    def __post_init__(self) -> None:
        if type(self.max_bytes) is not int or self.max_bytes <= 0:
            raise ValueError("max_bytes must be a positive integer")
        if self.max_bytes > (1 << 63) - 1:
            raise ValueError("max_bytes exceeds the native range")


@dataclass(slots=True)
class PlannerTrace:
    """Python-owned, columnar event log for offline visualization."""

    kind: Literal["discrete", "continuous"]
    events: NDArray[Any]
    points: NDArray[np.float64] | None = None
    node_labels: Sequence[Any] | None = None
    truncated: bool = False
    graph_bytes: int = 0

    def __post_init__(self) -> None:
        if self.events.dtype != TRACE_EVENT_DTYPE or self.events.ndim != 1:
            raise ValueError("events must use the native trace event dtype")
        if self.points is not None and self.points.ndim != 2:
            raise ValueError("points must be a 2D coordinate array")


__all__ = ["PlannerTrace", "TRACE_EVENT_DTYPE", "TraceOptions"]
