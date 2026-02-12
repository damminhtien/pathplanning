"""Nearest-neighbor index interfaces for planners."""

from __future__ import annotations

from collections.abc import Callable, Sequence
from typing import Any, Protocol, TypeAlias

import numpy as np

from .types import FloatArray

ScipyKDTree: TypeAlias = Any

Point: TypeAlias = FloatArray


def _as_point(point: Sequence[float] | FloatArray, dim: int) -> FloatArray:
    array = np.asarray(point, dtype=float)
    if array.shape != (dim,):
        raise ValueError(f"point must be shape ({dim},), got {array.shape}")
    return array


class NearestNeighborIndex(Protocol):
    """Contract for pluggable nearest-neighbor backends."""

    def add(self, point: Sequence[float] | FloatArray) -> int:
        """Insert ``point`` and return its index id."""
        ...

    def nearest(self, query: Sequence[float] | FloatArray) -> int:
        """Return the id of the nearest stored point to ``query``."""
        ...

    def radius(self, query: Sequence[float] | FloatArray, radius: float) -> list[int]:
        """Return ids of points within ``radius`` of ``query``."""
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
    """Vectorized NumPy nearest-neighbor index."""

    def __init__(
        self,
        dim: int,
        distance_fn: Callable[[FloatArray, FloatArray], float] | None = None,
    ) -> None:
        self._dim = dim
        self._distance_fn = distance_fn
        self._points: list[FloatArray] = []
        self._matrix: FloatArray | None = None
        self._dirty = False

    def add(self, point: Sequence[float] | FloatArray) -> int:
        array = _as_point(point, self._dim)
        self._points.append(array)
        self._dirty = True
        return len(self._points) - 1

    def _ensure_matrix(self) -> FloatArray:
        if self._matrix is None or self._dirty:
            if not self._points:
                self._matrix = np.empty((0, self._dim), dtype=float)
            else:
                self._matrix = np.vstack(self._points).astype(float, copy=False)
            self._dirty = False
        return self._matrix

    def nearest(self, query: Sequence[float] | FloatArray) -> int:
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

    def radius(self, query: Sequence[float] | FloatArray, radius: float) -> list[int]:
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
    import scipy.spatial as _scipy_spatial
except Exception:  # pragma: no cover - optional dependency
    _scipy_spatial = None
    from scipy.spatial import cKDTree as _cKDTree  # type: ignore[import-not-found]
except Exception:  # pragma: no cover - optional dependency
    _cKDTree = None


class KDTreeIndex:
    """Nearest-neighbor index backed by ``scipy.spatial.cKDTree``."""

    def __init__(self, dim: int) -> None:
        if _scipy_spatial is None or not hasattr(_scipy_spatial, "cKDTree"):
            raise ImportError("scipy is required for KDTreeIndex")
        self._dim = dim
        self._points: list[FloatArray] = []
        self._tree: ScipyKDTree | None = None
        if _cKDTree is None:
            raise ImportError("scipy is required for KDTreeIndex")
        self._dim = dim
        self._points: list[NDArray[np.float64]] = []
        self._tree: _KDTreeBackend | None = None
        self._dirty = False

    def add(self, point: Sequence[float] | FloatArray) -> int:
        array = _as_point(point, self._dim)
        self._points.append(array)
        self._dirty = True
        return len(self._points) - 1

    def _ensure_tree(self) -> ScipyKDTree:
        if self._tree is None or self._dirty:
            matrix = np.vstack(self._points).astype(float, copy=False)
            backend = getattr(_scipy_spatial, "cKDTree", None)
            if backend is None:
                raise RuntimeError("KDTree backend not available")
            self._tree = backend(matrix)
    def _ensure_tree(self) -> _KDTreeBackend:
        if _cKDTree is None:
            raise ImportError("scipy is required for KDTreeIndex")
        if self._tree is None or self._dirty:
            matrix = np.vstack(self._points).astype(float, copy=False)
            self._tree = cast(_KDTreeBackend, _cKDTree(matrix))
            self._dirty = False
        if self._tree is None:  # Defensive narrowing for type-checkers.
            raise RuntimeError("KDTree backend not initialized")
        return self._tree

    def nearest(self, query: Sequence[float] | FloatArray) -> int:
        if not self._points:
            raise ValueError("nearest() called on empty index")
        query_array = _as_point(query, self._dim)
        tree = self._ensure_tree()
        _distance, idx = tree.query(query_array, k=1)
        return int(idx)

    def radius(self, query: Sequence[float] | FloatArray, radius: float) -> list[int]:
        if not self._points:
            return []
        query_array = _as_point(query, self._dim)
        tree = self._ensure_tree()
        indices = tree.query_ball_point(query_array, r=float(radius))
        return [int(idx) for idx in indices]
