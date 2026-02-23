"""Reference 3D voxel-grid graph implementation for discrete search planners."""

from __future__ import annotations

from collections.abc import Iterable, Sequence
import math

Node3D = tuple[int, int, int]
Motion3D = tuple[int, int, int]

_DEFAULT_26_CONNECTED_MOTIONS: tuple[Motion3D, ...] = tuple(
    (dx, dy, dz)
    for dx in (-1, 0, 1)
    for dy in (-1, 0, 1)
    for dz in (-1, 0, 1)
    if (dx, dy, dz) != (0, 0, 0)
)


class Grid3DSearchSpace:
    """Finite 26-connected voxel graph with optional blocked cells."""

    def __init__(
        self,
        width: int = 21,
        height: int = 21,
        depth: int = 6,
        motions: Sequence[Motion3D] | None = None,
        obstacles: Iterable[Node3D] | None = None,
    ) -> None:
        self.x_range = int(width)
        self.y_range = int(height)
        self.z_range = int(depth)
        if self.x_range <= 0 or self.y_range <= 0 or self.z_range <= 0:
            raise ValueError("width, height, and depth must be > 0")

        chosen_motions = _DEFAULT_26_CONNECTED_MOTIONS if motions is None else tuple(motions)
        if not chosen_motions:
            raise ValueError("motions must be non-empty")
        self.motions: tuple[Motion3D, ...] = tuple(
            (int(dx), int(dy), int(dz)) for dx, dy, dz in chosen_motions
        )

        self.obs: set[Node3D] = set()
        if obstacles is not None:
            self.update_obs(obstacles)

    @property
    def obstacles(self) -> set[Node3D]:
        return self.obs

    def update_obs(self, obs: Iterable[Node3D]) -> None:
        self.obs = {self._coerce_node(node) for node in obs}

    @staticmethod
    def _coerce_node(node: Sequence[int] | Node3D) -> Node3D:
        if len(node) != 3:
            raise ValueError("node must have length 3")
        return int(node[0]), int(node[1]), int(node[2])

    def is_valid_node(self, n: Sequence[int] | Node3D) -> bool:
        node = self._coerce_node(n)
        x_coord, y_coord, z_coord = node
        in_bounds = (
            0 <= x_coord < self.x_range
            and 0 <= y_coord < self.y_range
            and 0 <= z_coord < self.z_range
        )
        return in_bounds and node not in self.obs

    def neighbors(self, n: Sequence[int] | Node3D) -> Iterable[Node3D]:
        node = self._coerce_node(n)
        for dx, dy, dz in self.motions:
            nxt = (node[0] + dx, node[1] + dy, node[2] + dz)
            if self.is_valid_node(nxt):
                yield nxt

    def edge_cost(self, a: Sequence[int] | Node3D, b: Sequence[int] | Node3D) -> float:
        src = self._coerce_node(a)
        dst = self._coerce_node(b)
        if not self.is_valid_node(src) or not self.is_valid_node(dst):
            return float("inf")

        dx = abs(dst[0] - src[0])
        dy = abs(dst[1] - src[1])
        dz = abs(dst[2] - src[2])
        if dx > 1 or dy > 1 or dz > 1 or (dx == 0 and dy == 0 and dz == 0):
            return float("inf")

        return math.sqrt(float(dx * dx + dy * dy + dz * dz))

    def heuristic(self, n: Sequence[int] | Node3D, goal: Sequence[int] | Node3D) -> float:
        node = self._coerce_node(n)
        target = self._coerce_node(goal)
        return math.sqrt(
            float(
                (target[0] - node[0]) ** 2 + (target[1] - node[1]) ** 2 + (target[2] - node[2]) ** 2
            )
        )


__all__ = ["Node3D", "Motion3D", "Grid3DSearchSpace"]
