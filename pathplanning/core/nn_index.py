"""Nearest-neighbor index interfaces for planners."""

from __future__ import annotations

import importlib
from typing import TYPE_CHECKING, Protocol, cast

import numpy as np
from numpy.typing import NDArray

from pathplanning.core.types import Mat, NodeId, Vec

if TYPE_CHECKING:  # pragma: no cover - typing-only optional import
    try:
        from scipy.spatial import cKDTree as _ScipyKDTree  # pyright: ignore[reportMissingTypeStubs, reportAttributeAccessIssue, reportUnknownVariableType]
    except Exception:  # pragma: no cover - scipy optional for typing

        class _ScipyKDTree:  # noqa: D101
            def __init__(self, *_args: object, **_kwargs: object) -> None:
                pass
else:  # pragma: no cover - runtime placeholder for optional dependency type

    class _ScipyKDTree:  # noqa: D101
        def __init__(self, *_args: object, **_kwargs: object) -> None:
            pass


def _as_point(point: Vec, dim: int) -> Vec:
    array = np.asarray(point, dtype=np.float64)
    if array.shape != (dim,):
        raise ValueError(f"point must be shape ({dim},), got {array.shape}")
    return array


def _as_matrix(points: Mat, dim: int) -> Mat:
    array = np.asarray(points, dtype=np.float64)
    if array.ndim != 2 or array.shape[1] != dim:
        raise ValueError(f"points must be shape (n, {dim}), got {array.shape}")
    return array


class NearestNeighborIndex(Protocol):
    """Contract for pluggable nearest-neighbor backends."""

    def build(self, points: Mat) -> None:
        """Build or rebuild the index from planner points."""
        ...

    def nearest(self, q: Vec) -> NodeId:
        """Return the id of the nearest stored point to ``q``."""
        ...

    def radius(self, q: Vec, r: float) -> NDArray[np.int64]:
        """Return ids of points within radius ``r`` around ``q``."""
        ...


class NaiveNnIndex:
    """Vectorized NumPy nearest-neighbor index."""

    def __init__(self, dim: int) -> None:
        self._dim = dim
        self._points: Mat = np.empty((0, dim), dtype=np.float64)

    def build(self, points: Mat) -> None:
        self._points = _as_matrix(points, self._dim).copy()

    def nearest(self, q: Vec) -> NodeId:
        if self._points.shape[0] == 0:
            raise ValueError("nearest() called on empty index")
        query = _as_point(q, self._dim)
        deltas = self._points - query
        distances_sq = np.einsum("ij,ij->i", deltas, deltas)
        return int(np.argmin(distances_sq))

    def radius(self, q: Vec, r: float) -> NDArray[np.int64]:
        if r < 0:
            raise ValueError("radius must be non-negative")
        if self._points.shape[0] == 0:
            return np.empty((0,), dtype=np.int64)
        query = _as_point(q, self._dim)
        deltas = self._points - query
        distances_sq = np.einsum("ij,ij->i", deltas, deltas)
        threshold = float(r) ** 2
        return np.where(distances_sq <= threshold)[0].astype(np.int64, copy=False)


class _KDTreeBackend(Protocol):
    """Typed subset of ``scipy.spatial.cKDTree`` used by this module."""

    def query(
        self, x: Vec, k: int = 1
    ) -> tuple[float, int] | tuple[NDArray[np.float64], NDArray[np.int64]]:
        """Return nearest-neighbor distance and index."""
        ...

    def query_ball_point(self, x: Vec, r: float) -> list[int]:
        """Return indices inside the search radius."""
        ...


try:  # pragma: no cover - optional runtime dependency
    _scipy_spatial = importlib.import_module("scipy.spatial")
    _cKDTree_runtime = cast(type[_ScipyKDTree], _scipy_spatial.cKDTree)
except Exception:  # pragma: no cover - scipy optional
    _cKDTree_runtime: type[_ScipyKDTree] | None = None


class KDTreeNnIndex:
    """Nearest-neighbor index backed by ``scipy.spatial.cKDTree``."""

    def __init__(self, dim: int) -> None:
        self._dim = dim
        self._points: Mat = np.empty((0, dim), dtype=np.float64)
        self._tree: _KDTreeBackend | None = None

    def _runtime_kdtree_class(self) -> type[_ScipyKDTree]:
        runtime_kdtree = _cKDTree_runtime
        if runtime_kdtree is None:
            raise RuntimeError("scipy is required for KDTreeNnIndex")
        return runtime_kdtree

    def build(self, points: Mat) -> None:
        runtime_kdtree = self._runtime_kdtree_class()
        self._points = _as_matrix(points, self._dim).copy()
        if self._points.shape[0] == 0:
            self._tree = None
            return
        self._tree = cast(_KDTreeBackend, runtime_kdtree(self._points))

    def nearest(self, q: Vec) -> NodeId:
        self._runtime_kdtree_class()
        if self._tree is None:
            raise ValueError("nearest() called on empty index")
        query = _as_point(q, self._dim)
        _, index = self._tree.query(query, k=1)
        if isinstance(index, np.ndarray):
            return int(index.item())
        return int(index)

    def radius(self, q: Vec, r: float) -> NDArray[np.int64]:
        self._runtime_kdtree_class()
        if r < 0:
            raise ValueError("radius must be non-negative")
        if self._tree is None:
            return np.empty((0,), dtype=np.int64)
        query = _as_point(q, self._dim)
        indices = self._tree.query_ball_point(query, r=float(r))
        return np.asarray(indices, dtype=np.int64)


# Stable aliases for existing imports.
NaiveIndex = NaiveNnIndex
KDTreeIndex = KDTreeNnIndex
