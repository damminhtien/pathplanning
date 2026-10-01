"""Native CSR graphs for discrete planners."""

from __future__ import annotations

from collections.abc import Callable, Iterable, Sequence
import ctypes
import operator
from typing import Generic, Hashable, Literal, TypeVar, cast
import weakref

import numpy as np
from numpy.typing import NDArray

from pathplanning.native._ffi import load_native_library

N = TypeVar("N", bound=Hashable)
Heuristic = Callable[[N, N], float]


class NativeGraphError(ValueError):
    """Raised when CSR input cannot be represented by the native engine."""


def _integer_array(values: Sequence[int] | NDArray[np.integer], name: str) -> NDArray[np.uint64]:
    array = np.asarray(values)
    if array.ndim != 1 or array.dtype.kind not in "iu":
        raise NativeGraphError(f"{name} must be a one-dimensional integer array")
    if array.dtype.kind == "i" and bool(np.any(array < 0)):
        raise NativeGraphError(f"{name} cannot contain negative values")
    try:
        return np.ascontiguousarray(array, dtype=np.uint64)
    except (OverflowError, ValueError) as exc:
        raise NativeGraphError(f"{name} values must fit in uint64") from exc


def _make_label_index(labels: Sequence[N]) -> dict[N, int]:
    index: dict[N, int] = {}
    for node_id, label in enumerate(labels):
        if label in index:
            raise NativeGraphError(f"node labels must be unique; duplicate label: {label!r}")
        index[label] = node_id
    return index


class _IntegerNodeLabels(Sequence[int]):
    """Represent the default integer IDs without allocating one Python int per node."""

    def __init__(self, node_count: int) -> None:
        self._node_count = node_count

    def __len__(self) -> int:
        return self._node_count

    def __getitem__(self, node_id: int | slice) -> int | tuple[int, ...]:
        if isinstance(node_id, slice):
            return tuple(range(*node_id.indices(self._node_count)))
        if node_id < 0:
            node_id += self._node_count
        if node_id < 0 or node_id >= self._node_count:
            raise IndexError(node_id)
        return node_id

    def node_id(self, node: object) -> int | None:
        try:
            node_id = operator.index(node)
        except TypeError:
            return None
        return node_id if 0 <= node_id < self._node_count else None


class _GridNodeLabels(Sequence[tuple[int, ...]]):
    """Map flat grid IDs to coordinates without a per-node Python dictionary."""

    def __init__(self, width: int, height: int, depth: int, dimensions: int) -> None:
        self.width = width
        self.height = height
        self.depth = depth
        self.dimensions = dimensions
        self._node_count = width * height * depth

    def __len__(self) -> int:
        return self._node_count

    def __getitem__(self, node_id: int | slice) -> tuple[int, ...] | tuple[tuple[int, ...], ...]:
        if isinstance(node_id, slice):
            return tuple(self[index] for index in range(*node_id.indices(self._node_count)))
        if node_id < 0:
            node_id += self._node_count
        if node_id < 0 or node_id >= self._node_count:
            raise IndexError(node_id)
        z_coord, remainder = divmod(node_id, self.width * self.height)
        y_coord, x_coord = divmod(remainder, self.width)
        if self.dimensions == 2:
            return x_coord, y_coord
        return x_coord, y_coord, z_coord

    def node_id(self, node: object) -> int | None:
        try:
            coordinates = tuple(int(value) for value in node)  # type: ignore[arg-type]
        except (TypeError, ValueError, OverflowError):
            return None
        if len(coordinates) != self.dimensions:
            return None
        x_coord, y_coord = coordinates[:2]
        z_coord = coordinates[2] if self.dimensions == 3 else 0
        if not (
            0 <= x_coord < self.width and 0 <= y_coord < self.height and 0 <= z_coord < self.depth
        ):
            return None
        return x_coord + self.width * (y_coord + self.height * z_coord)


class NativeGraph(Generic[N]):
    """An immutable graph stored as native C++ CSR arrays.

    Use :meth:`from_csr` for large graphs or :meth:`from_edges` for small
    hand-built graphs. Node labels stay in Python; adjacency and search state
    stay in C++.
    """

    def __init__(
        self,
        indptr: Sequence[int] | NDArray[np.integer],
        indices: Sequence[int] | NDArray[np.integer],
        weights: Sequence[float] | NDArray[np.floating],
        *,
        node_labels: Sequence[N] | None = None,
        heuristic: Heuristic[N] | None = None,
    ) -> None:
        offsets = _integer_array(indptr, "indptr")
        neighbor_ids = _integer_array(indices, "indices")
        costs = np.ascontiguousarray(weights, dtype=np.float64)
        if costs.ndim != 1:
            raise NativeGraphError("weights must be a one-dimensional array")
        if offsets.size < 2:
            raise NativeGraphError("CSR graph must contain at least one node")
        node_count = int(offsets.size - 1)
        edge_count = int(neighbor_ids.size)
        if costs.size != edge_count:
            raise NativeGraphError("indices and weights must have the same length")
        if int(offsets[0]) != 0 or int(offsets[-1]) != edge_count:
            raise NativeGraphError("indptr must start at zero and end at the edge count")
        if bool(np.any(offsets[1:] < offsets[:-1])):
            raise NativeGraphError("indptr must be monotonically non-decreasing")
        if bool(np.any(neighbor_ids >= node_count)):
            raise NativeGraphError("indices contains a node id outside the graph")

        labels: Sequence[N]
        if node_labels is None:
            labels = cast(Sequence[N], _IntegerNodeLabels(node_count))
            label_to_id = None
        else:
            labels = tuple(node_labels)
            label_to_id = _make_label_index(labels)
        if len(labels) != node_count:
            raise NativeGraphError("node_labels length must match the number of CSR rows")

        library = load_native_library()
        handle = ctypes.c_void_p()
        error_buffer = ctypes.create_string_buffer(512)
        offsets_pointer = offsets.ctypes.data_as(ctypes.POINTER(ctypes.c_uint64))
        ids_pointer = neighbor_ids.ctypes.data_as(ctypes.POINTER(ctypes.c_uint64))
        costs_pointer = costs.ctypes.data_as(ctypes.POINTER(ctypes.c_double))
        status = int(
            library.pp_graph_create_csr(
                ctypes.c_uint64(node_count),
                ctypes.c_uint64(edge_count),
                offsets_pointer,
                ids_pointer,
                costs_pointer,
                ctypes.byref(handle),
                error_buffer,
                ctypes.c_size_t(len(error_buffer)),
            )
        )
        if status != 0 or handle.value is None:
            message = error_buffer.value.decode("utf-8", errors="replace")
            raise NativeGraphError(message or "native graph creation failed")

        self._adopt_handle(library, handle, labels, label_to_id, heuristic)

    def _adopt_handle(
        self,
        library: ctypes.CDLL,
        handle: ctypes.c_void_p,
        labels: Sequence[N],
        label_to_id: dict[N, int] | None,
        heuristic: Heuristic[N] | None,
        heuristic_mode: Literal["euclidean"] | None = None,
    ) -> None:
        self._library = library
        self._handle = handle
        self._finalizer = weakref.finalize(self, library.pp_graph_free, handle)
        self._node_labels = labels
        self._node_to_id = label_to_id
        self._grid_labels = labels if isinstance(labels, _GridNodeLabels) else None
        self._heuristic_fn = heuristic
        self._heuristic_mode = heuristic_mode

    @classmethod
    def from_csr(
        cls,
        indptr: Sequence[int] | NDArray[np.integer],
        indices: Sequence[int] | NDArray[np.integer],
        weights: Sequence[float] | NDArray[np.floating],
        *,
        node_labels: Sequence[N] | None = None,
        heuristic: Heuristic[N] | None = None,
    ) -> NativeGraph[N]:
        """Copy CSR arrays into a native graph.

        The three arrays can come directly from SciPy CSR objects without
        making SciPy a runtime dependency: pass ``matrix.indptr``,
        ``matrix.indices``, and ``matrix.data``.
        """
        return cls(
            indptr,
            indices,
            weights,
            node_labels=node_labels,
            heuristic=heuristic,
        )

    @classmethod
    def from_edges(
        cls,
        nodes: Sequence[N],
        edges: Iterable[tuple[N, N, float]],
        *,
        directed: bool = True,
        heuristic: Heuristic[N] | None = None,
    ) -> NativeGraph[N]:
        """Build a native graph from labeled weighted edges.

        For an undirected graph each edge is inserted in both directions.
        ``nodes`` includes isolated nodes and defines deterministic ID order.
        """
        labels = tuple(nodes)
        node_to_id = _make_label_index(labels)
        adjacency: list[list[tuple[int, float]]] = [[] for _ in labels]

        for source, target, weight in edges:
            try:
                source_id = node_to_id[source]
                target_id = node_to_id[target]
            except KeyError as exc:
                raise NativeGraphError(
                    f"edge endpoint is missing from nodes: {exc.args[0]!r}"
                ) from exc
            cost = float(weight)
            adjacency[source_id].append((target_id, cost))
            if not directed and source_id != target_id:
                adjacency[target_id].append((source_id, cost))

        offsets = [0]
        neighbor_ids: list[int] = []
        costs: list[float] = []
        for row in adjacency:
            for neighbor_id, cost in row:
                neighbor_ids.append(neighbor_id)
                costs.append(cost)
            offsets.append(len(neighbor_ids))

        return cls.from_csr(
            offsets,
            neighbor_ids,
            costs,
            node_labels=labels,
            heuristic=heuristic,
        )

    @classmethod
    def from_grid(
        cls,
        *,
        dimensions: int,
        width: int,
        height: int,
        depth: int,
        motions: Sequence[Sequence[int]],
        valid_nodes: Sequence[bool] | NDArray[np.bool_] | None = None,
        node_labels: Sequence[N] | None = None,
        heuristic: Heuristic[N] | None = None,
        heuristic_mode: Literal["euclidean"] | None = None,
    ) -> NativeGraph[N]:
        """Build regular 2D or 3D grid adjacency directly in C++.

        Omit ``valid_nodes`` when every grid cell is valid. Set
        ``heuristic_mode="euclidean"`` to compute Euclidean heuristics in the
        native search kernel without materializing one value per node.
        """
        if dimensions not in (2, 3):
            raise NativeGraphError("grid must have two or three dimensions")
        if heuristic_mode not in (None, "euclidean"):
            raise NativeGraphError("heuristic_mode must be None or 'euclidean'")
        if width <= 0 or height <= 0 or depth <= 0 or (dimensions == 2 and depth != 1):
            raise NativeGraphError("grid dimensions must be positive and match dimensionality")

        node_count = width * height * depth
        if node_labels is None:
            labels: Sequence[N] = cast(
                Sequence[N], _GridNodeLabels(width, height, depth, dimensions)
            )
            label_to_id = None
        else:
            labels = tuple(node_labels)
            if len(labels) != node_count:
                raise NativeGraphError("node_labels length must match the number of grid nodes")
            label_to_id = _make_label_index(labels)

        valid_array = (
            None if valid_nodes is None else np.ascontiguousarray(valid_nodes, dtype=np.uint8)
        )
        if valid_array is not None and (valid_array.ndim != 1 or valid_array.size != node_count):
            raise NativeGraphError("valid_nodes length must match the number of grid nodes")

        motion_rows: list[tuple[int, int, int]] = []
        for motion in motions:
            if len(motion) != dimensions:
                raise NativeGraphError("motion dimensions must match the grid")
            values = tuple(int(value) for value in motion)
            if any(
                value < np.iinfo(np.int32).min or value > np.iinfo(np.int32).max for value in values
            ):
                raise NativeGraphError("motion values must fit in int32")
            motion_rows.append((values[0], values[1], values[2] if dimensions == 3 else 0))
        if not motion_rows:
            raise NativeGraphError("grid motions must not be empty")

        motion_array = np.ascontiguousarray(motion_rows, dtype=np.int32)
        library = load_native_library()
        handle = ctypes.c_void_p()
        error_buffer = ctypes.create_string_buffer(512)
        motion_pointer = motion_array.ctypes.data_as(ctypes.POINTER(ctypes.c_int32))
        valid_pointer = (
            valid_array.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8))
            if valid_array is not None
            else ctypes.POINTER(ctypes.c_uint8)()
        )
        status = int(
            library.pp_graph_create_grid_ex(
                ctypes.c_uint64(width),
                ctypes.c_uint64(height),
                ctypes.c_uint64(depth),
                ctypes.c_uint64(dimensions),
                ctypes.c_int(1 if heuristic_mode == "euclidean" else 0),
                motion_pointer,
                ctypes.c_size_t(len(motion_rows)),
                valid_pointer,
                ctypes.byref(handle),
                error_buffer,
                ctypes.c_size_t(len(error_buffer)),
            )
        )
        if status != 0 or handle.value is None:
            message = error_buffer.value.decode("utf-8", errors="replace")
            raise NativeGraphError(message or "native grid graph creation failed")

        graph = cls.__new__(cls)
        graph._adopt_handle(
            library,
            handle,
            labels,
            label_to_id,
            heuristic,
            heuristic_mode,
        )
        return graph

    @property
    def node_count(self) -> int:
        """Return the number of labeled vertices."""
        return len(self._node_labels)

    @property
    def node_labels(self) -> Sequence[N]:
        """Return labels in native node-ID order."""
        return self._node_labels

    @property
    def _native_handle(self) -> ctypes.c_void_p:
        if not self._finalizer.alive:
            raise RuntimeError("native graph is closed")
        return self._handle

    def _node_id(self, node: N) -> int | None:
        node_to_id = self._node_to_id
        if node_to_id is not None:
            return node_to_id.get(node)
        if isinstance(self._node_labels, _IntegerNodeLabels):
            return self._node_labels.node_id(node)
        grid_labels = self._grid_labels
        return None if grid_labels is None else grid_labels.node_id(node)

    def _node_for_id(self, node_id: int) -> N:
        return self._node_labels[node_id]

    def _heuristic(self, node_id: int, goal: N) -> float:
        if self._heuristic_fn is None:
            return 0.0
        return float(self._heuristic_fn(self._node_for_id(node_id), goal))

    def close(self) -> None:
        """Release the native graph handle; subsequent searches will fail."""
        self._finalizer()

    def __enter__(self) -> NativeGraph[N]:
        return self

    def __exit__(self, *_: object) -> None:
        self.close()


__all__ = ["NativeGraph", "NativeGraphError"]
