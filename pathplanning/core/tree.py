"""Array-backed tree storage for scalable planners."""

from __future__ import annotations

from collections.abc import Sequence

import numpy as np
from numpy.typing import NDArray

from pathplanning.core.types import FloatArray, Mat, NodeId, Vec


class ArrayTree:
    """Array-backed tree storage for planners.

    Stores node coordinates, parent indices, and cumulative costs in NumPy arrays.
    """

    def __init__(self, dim: int) -> None:
        if dim <= 0:
            raise ValueError("dim must be positive")
        self._dim = dim
        self.nodes: FloatArray = np.empty((0, dim), dtype=np.float64)
        self.parent: NDArray[np.int32] = np.empty((0,), dtype=np.int32)
        self.cost: FloatArray = np.empty((0,), dtype=np.float64)

    @property
    def size(self) -> int:
        """Return number of nodes in the tree."""
        return int(self.nodes.shape[0])

    def add_node(self, x: Sequence[float] | Vec, parent: NodeId, cost: float) -> NodeId:
        """Append a node and return its index id.

        Args:
            x: Node coordinate with shape ``(dim,)``.
            parent: Parent node index or ``-1`` for root.
            cost: Cumulative cost-to-come for the node.
        """
        point = np.asarray(x, dtype=np.float64)
        assert point.ndim == 1, "x must be 1D"
        assert point.shape == (self._dim,), f"x must be shape ({self._dim},), got {point.shape}"
        self.nodes = np.vstack((self.nodes, point.reshape(1, self._dim)))
        self.parent = np.concatenate(
            (self.parent, np.asarray([int(parent)], dtype=np.int32)),
            axis=0,
        )
        self.cost = np.concatenate(
            (self.cost, np.asarray([float(cost)], dtype=np.float64)),
            axis=0,
        )
        return self.size - 1

    def node(self, index: NodeId) -> Vec:
        """Return node coordinates at ``index``."""
        if index < 0 or index >= self.size:
            raise IndexError("index out of bounds")
        return self.nodes[int(index)]

    def extract_path(self, node_id: NodeId) -> Mat:
        """Return the path from root to ``node_id`` as an array."""
        if node_id < 0 or node_id >= self.size:
            raise IndexError("node_id out of bounds")
        indices: list[int] = []
        current = int(node_id)
        while current != -1:
            indices.append(current)
            current = int(self.parent[current])
        indices.reverse()
        return self.nodes[np.asarray(indices, dtype=int)]


# Keep a stable alias for existing imports.
Tree = ArrayTree
