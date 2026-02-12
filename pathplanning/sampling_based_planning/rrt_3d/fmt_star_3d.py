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

from pathplanning.utils.priority_queue import MinHeapPriorityQueue

from pathplanning.spaces.environment3d import Environment3D
from .utils_3d import get_dist, is_collide, sample_free


class FmtStar:
    """Fast Marching Tree* planner for 3D environments."""

    def __init__(self, radius: float = 1, n: int = 1000, rng: np.random.Generator | None = None) -> None:
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
            self.v_open,
            self.v_open_queue,
            self.v_closed,
            self.vertices,
            self.v_unvisited,
            self.c,
        ) = self.init_node_sets()
        self.neighbors: dict[tuple[float, ...], set[tuple[float, ...]]] = {}
        self.done = True
        self.path_edges: list[Any] = []
        self.parent_by_node: dict[tuple[float, ...], tuple[float, ...]] = {}
        self.rng = rng if rng is not None else np.random.default_rng()

    def generate_sample_set(self, n: int) -> set[tuple[float, ...]]:
        """Generate collision-free sample set.

        Args:
            n: Number of samples.

        Returns:
            Set of sampled states.
        """
        vertices: set[tuple[float, ...]] = set()
        for _ in range(n):
            vertices.add(tuple(sample_free(self, bias=0.0, rng=self.rng)))
        return vertices

    def init_node_sets(
        self,
    ) -> tuple[
        set[tuple[float, ...]],
        MinHeapPriorityQueue,
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

        v_open_queue = MinHeapPriorityQueue()
        v_open_queue.put(self.xinit, costs[self.xinit])
        return v_open, v_open_queue, v_closed, vertices, v_unvisited, costs

    def near(
        self, nodeset: set[tuple[float, ...]], node: tuple[float, ...], radius: float
    ) -> set[tuple[float, ...]]:
        """Return neighbors within radius."""
        if node in self.neighbors:
            return self.neighbors[node]
        return {candidate for candidate in nodeset if get_dist(candidate, node) < radius}

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
            path_edges.append((state, self.parent_by_node[state]))
            state = self.parent_by_node[state]
            if i > self.n:
                break
            i += 1
        return path_edges

    def cost(self, x: tuple[float, ...], y: tuple[float, ...]) -> float:
        """Return edge metric used by FMT*."""
        return get_dist(x, y)

    def run(self) -> None:
        """Run FMT* search until goal or failure."""
        z = self.xinit
        radius = self.radius
        nz = self.near(self.v_unvisited, z, radius)
        edges: set[tuple[tuple[float, ...], tuple[float, ...]]] = set()
        self.save_neighbors(nz, z)
        ind = 0
        while z != self.xgoal:
            v_open_new: set[tuple[float, ...]] = set()
            x_near = self.near(self.v_unvisited, z, radius)
            self.save_neighbors(x_near, z)
            for x in x_near:
                y_near = list(self.near(self.v_open, x, radius))
                ymin = y_near[np.argmin([self.c[y] + self.cost(y, x) for y in y_near])]
                collide, _ = is_collide(self, ymin, x)
                if not collide:
                    edges.add((ymin, x))
                    v_open_new.add(x)
                    self.parent_by_node[x] = z
                    self.v_unvisited = self.v_unvisited.difference({x})
                    self.c[x] = self.c[ymin] + self.cost(ymin, x)
            self.v_open = self.v_open.union(v_open_new).difference({z})
            self.v_closed.add(z)
            if len(self.v_open) == 0:
                print("Failure")
                return
            ind += 1
            print(str(ind) + " node expanded")
            v_open_list = list(self.v_open)
            z = v_open_list[np.argmin([self.c[y] for y in self.v_open])]

        tree = (self.v_open.union(self.v_closed), edges)
        self.done = True
        self.path_edges = self.path(z, tree)


if __name__ == "__main__":
    planner = FmtStar(radius=1, n=3000)
    planner.run()
