"""Nearest-neighbor index interfaces for planners."""

from __future__ import annotations

from collections.abc import Callable, Sequence
from typing import Protocol, cast

import numpy as np
from numpy.typing import NDArray

Point = NDArray[np.float64]


def _as_point(point: Sequence[float] | NDArray[np.float64], dim: int) -> NDArray[np.float64]:
    array = np.asarray(point, dtype=float)
    if array.shape != (dim,):
        raise ValueError(f"point must be shape ({dim},), got {array.shape}")
    return array


class NearestNeighborIndex(Protocol):
    """Contract for pluggable nearest-neighbor backends."""

    def add(self, point: Sequence[float] | NDArray[np.float64]) -> int:
        """Insert ``point`` and return its index id."""
        ...

    def nearest(self, query: Sequence[float] | NDArray[np.float64]) -> int:
        """Return the id of the nearest stored point to ``query``."""
        ...

    def radius(self, query: Sequence[float] | NDArray[np.float64], radius: float) -> list[int]:
        """Return ids of points within ``radius`` of ``query``."""
        ...


class _KDTreeBackend(Protocol):
    """Typed subset of ``scipy.spatial.cKDTree`` used by this module."""

    def query(
        self, x: NDArray[np.float64], k: int = 1
    ) -> tuple[float | NDArray[np.float64], int | NDArray[np.int64]]:
        """Return nearest-neighbor distance and index."""
        ...

    def query_ball_point(self, x: NDArray[np.float64], r: float) -> list[int]:
        """Return indices inside the search radius."""
        ...


class NaiveIndex:
    """Vectorized NumPy nearest-neighbor index.

    Uses Euclidean distance when no custom distance function is provided.
    """

    def __init__(
        self,
        dim: int,
        distance_fn: Callable[[NDArray[np.float64], NDArray[np.float64]], float] | None = None,
    ) -> None:
        self._dim = dim
        self._distance_fn = distance_fn
        self._points: list[NDArray[np.float64]] = []
        self._matrix: NDArray[np.float64] | None = None
        self._dirty = False

    def add(self, point: Sequence[float] | NDArray[np.float64]) -> int:
        array = _as_point(point, self._dim)
        self._points.append(array)
        self._dirty = True
        return len(self._points) - 1

    def _ensure_matrix(self) -> NDArray[np.float64]:
        if self._matrix is None or self._dirty:
            if not self._points:
                self._matrix = np.empty((0, self._dim), dtype=float)
            else:
                self._matrix = np.vstack(self._points).astype(float, copy=False)
            self._dirty = False
        return self._matrix

    def nearest(self, query: Sequence[float] | NDArray[np.float64]) -> int:
        if not self._points:
            raise ValueError("nearest() called on empty index")
        query_array = _as_point(query, self._dim)
        if self._distance_fn is not None:
            distances = [self._distance_fn(point, query_array) for point in self._points]
            return int(np.argmin(np.asarray(distances, dtype=float)))

        matrix = self._ensure_matrix()
        deltas = matrix - query_array
        distances = np.einsum("ij,ij->i", deltas, deltas)
        return int(np.argmin(distances))

    def radius(self, query: Sequence[float] | NDArray[np.float64], radius: float) -> list[int]:
        if not self._points:
            return []
        query_array = _as_point(query, self._dim)
        if self._distance_fn is not None:
            indices: list[int] = []
            for idx, point in enumerate(self._points):
                if self._distance_fn(point, query_array) <= radius:
                    indices.append(idx)
            return indices

        matrix = self._ensure_matrix()
        deltas = matrix - query_array
        distances = np.einsum("ij,ij->i", deltas, deltas)
        radius_sq = float(radius) ** 2
        return [int(idx) for idx in np.where(distances <= radius_sq)[0]]


try:  # Optional SciPy backend.
    from scipy.spatial import cKDTree as _cKDTree  # type: ignore[import-not-found]
except Exception:  # pragma: no cover - optional dependency
    _cKDTree = None


class KDTreeIndex:
    """Nearest-neighbor index backed by ``scipy.spatial.cKDTree``."""

    def __init__(self, dim: int) -> None:
        if _cKDTree is None:
            raise ImportError("scipy is required for KDTreeIndex")
        self._dim = dim
        self._points: list[NDArray[np.float64]] = []
        self._tree: _KDTreeBackend | None = None
        self._dirty = False

    def add(self, point: Sequence[float] | NDArray[np.float64]) -> int:
        array = _as_point(point, self._dim)
        self._points.append(array)
        self._dirty = True
        return len(self._points) - 1

    def _ensure_tree(self) -> _KDTreeBackend:
        if _cKDTree is None:
            raise ImportError("scipy is required for KDTreeIndex")
        if self._tree is None or self._dirty:
            matrix = np.vstack(self._points).astype(float, copy=False)
            self._tree = cast(_KDTreeBackend, _cKDTree(matrix))
            self._dirty = False
        return self._tree

    def nearest(self, query: Sequence[float] | NDArray[np.float64]) -> int:
        if not self._points:
            raise ValueError("nearest() called on empty index")
        query_array = _as_point(query, self._dim)
        tree = self._ensure_tree()
        _, idx = tree.query(query_array, k=1)
        return int(idx)

    def radius(self, query: Sequence[float] | NDArray[np.float64], radius: float) -> list[int]:
        if not self._points:
            return []
        query_array = _as_point(query, self._dim)
        tree = self._ensure_tree()
        indices = tree.query_ball_point(query_array, r=float(radius))
        return [int(idx) for idx in indices]
