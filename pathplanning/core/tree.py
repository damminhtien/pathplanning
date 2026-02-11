"""Array-backed tree storage for scalable planners."""

from __future__ import annotations

from collections.abc import Sequence

import numpy as np
from numpy.typing import NDArray


class Tree:
    """Tree container with array-backed storage.

    Stores node coordinates, parent indices, and cumulative costs in NumPy arrays.
    """

    def __init__(self, dim: int) -> None:
        if dim <= 0:
            raise ValueError("dim must be positive")
        self._dim = dim
        self.nodes: NDArray[np.float64] = np.empty((0, dim), dtype=float)
        self.parent: NDArray[np.int64] = np.empty((0,), dtype=int)
        self.cost: NDArray[np.float64] = np.empty((0,), dtype=float)

    @property
    def size(self) -> int:
        """Return number of nodes in the tree."""
        return int(self.parent.shape[0])

    def append_node(
        self, x: Sequence[float] | NDArray[np.float64], parent_id: int, cost: float
    ) -> int:
        """Append a node and return its index id.

        Args:
            x: Node coordinate with shape ``(dim,)``.
            parent_id: Parent node index or ``-1`` for root.
            cost: Cumulative cost-to-come for the node.
        """
        point = np.asarray(x, dtype=float)
        if point.shape != (self._dim,):
            raise ValueError(f"x must be shape ({self._dim},), got {point.shape}")

        if self.nodes.size == 0:
            self.nodes = point.reshape(1, self._dim)
        else:
            self.nodes = np.vstack([self.nodes, point])

        self.parent = np.append(self.parent, int(parent_id))
        self.cost = np.append(self.cost, float(cost))
        return self.size - 1

    def extract_path(self, node_id: int) -> NDArray[np.float64]:
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
