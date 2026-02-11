"""Fast Marching Tree* implementation for 3D planning.

References:
    Janson, Lucas, et al. "Fast marching tree: A fast marching sampling-based
    method for optimal motion planning in many dimensions." The International
    Journal of Robotics Research 34.7 (2015): 883-921.
"""

from __future__ import annotations

import copy
from typing import Any

import numpy as np

from .env_3d import Environment3D
from .queue import MinheapPQ
from .utils_3d import getDist, isCollide, sampleFree


class FmtStar:
    """Fast Marching Tree* planner for 3D environments."""

    def __init__(self, radius: float = 1, n: int = 1000) -> None:
        """Initialize planner state.

        Args:
            radius: Neighbor search radius.
            n: Number of sampled states.
        """
        self.env = Environment3D()
        self.xinit, self.xgoal = tuple(self.env.start), tuple(self.env.goal)
        self.x0, self.xt = tuple(self.env.start), tuple(self.env.goal)
        self.n = n
        self.radius = radius
        (
            self.Vopen,
            self.Vopen_queue,
            self.Vclosed,
            self.V,
            self.Vunvisited,
            self.c,
        ) = self.init_node_sets()
        self.neighbors: dict[tuple[float, ...], set[tuple[float, ...]]] = {}
        self.done = True
        self.Path: list[Any] = []
        self.Parent: dict[tuple[float, ...], tuple[float, ...]] = {}

    def generate_sample_set(self, n: int) -> set[tuple[float, ...]]:
        """Generate collision-free sample set.

        Args:
            n: Number of samples.

        Returns:
            Set of sampled states.
        """
        vertices: set[tuple[float, ...]] = set()
        for _ in range(n):
            vertices.add(tuple(sampleFree(self, bias=0.0)))
        return vertices

    def init_node_sets(
        self,
    ) -> tuple[
        set[tuple[float, ...]],
        MinheapPQ,
        set[tuple[float, ...]],
        set[tuple[float, ...]],
        set[tuple[float, ...]],
        dict[tuple[float, ...], float],
    ]:
        """Initialize open/closed/unvisited sets and costs."""
        v_open = {self.xinit}
        v_closed: set[tuple[float, ...]] = set()
        vertices = self.generate_sample_set(self.n - 2)
        v_unvisited = copy.deepcopy(vertices)
        v_unvisited.add(self.xgoal)
        vertices.add(self.xinit)
        vertices.add(self.xgoal)

        costs = {node: np.inf for node in vertices}
        costs[self.xinit] = 0

        v_open_queue = MinheapPQ()
        v_open_queue.put(self.xinit, costs[self.xinit])
        return v_open, v_open_queue, v_closed, vertices, v_unvisited, costs

    def near(
        self, nodeset: set[tuple[float, ...]], node: tuple[float, ...], radius: float
    ) -> set[tuple[float, ...]]:
        """Return neighbors within radius."""
        if node in self.neighbors:
            return self.neighbors[node]
        return {candidate for candidate in nodeset if getDist(candidate, node) < radius}

    def save_neighbors(
        self, associated_nodes: set[tuple[float, ...]], node: tuple[float, ...]
    ) -> None:
        """Cache neighborhood for a node."""
        self.neighbors[node] = associated_nodes

    def path(
        self,
        _z: tuple[float, ...],
        _init_tree: tuple[set[tuple[float, ...]], set[tuple[tuple[float, ...], tuple[float, ...]]]],
    ) -> list[tuple[tuple[float, ...], tuple[float, ...]]]:
        """Extract path from goal to start using parent pointers."""
        path_edges: list[tuple[tuple[float, ...], tuple[float, ...]]] = []
        state = self.xgoal
        i = 0
        while state != self.xinit:
            path_edges.append((state, self.Parent[state]))
            state = self.Parent[state]
            if i > self.n:
                break
            i += 1
        return path_edges

    def cost(self, x: tuple[float, ...], y: tuple[float, ...]) -> float:
        """Return edge metric used by FMT*."""
        return getDist(x, y)

    def run(self) -> None:
        """Run FMT* search until goal or failure."""
        z = self.xinit
        radius = self.radius
        nz = self.near(self.Vunvisited, z, radius)
        edges: set[tuple[tuple[float, ...], tuple[float, ...]]] = set()
        self.save_neighbors(nz, z)
        ind = 0
        while z != self.xgoal:
            v_open_new: set[tuple[float, ...]] = set()
            x_near = self.near(self.Vunvisited, z, radius)
            self.save_neighbors(x_near, z)
            for x in x_near:
                y_near = list(self.near(self.Vopen, x, radius))
                ymin = y_near[np.argmin([self.c[y] + self.cost(y, x) for y in y_near])]
                collide, _ = isCollide(self, ymin, x)
                if not collide:
                    edges.add((ymin, x))
                    v_open_new.add(x)
                    self.Parent[x] = z
                    self.Vunvisited = self.Vunvisited.difference({x})
                    self.c[x] = self.c[ymin] + self.cost(ymin, x)
            self.Vopen = self.Vopen.union(v_open_new).difference({z})
            self.Vclosed.add(z)
            if len(self.Vopen) == 0:
                print("Failure")
                return
            ind += 1
            print(str(ind) + " node expanded")
            v_open_list = list(self.Vopen)
            z = v_open_list[np.argmin([self.c[y] for y in self.Vopen])]

        tree = (self.Vopen.union(self.Vclosed), edges)
        self.done = True
        self.Path = self.path(z, tree)


if __name__ == "__main__":
    planner = FmtStar(radius=1, n=3000)
    planner.run()
