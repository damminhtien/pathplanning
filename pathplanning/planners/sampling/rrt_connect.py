"""RRT-Connect implementation for 3D planning."""

from __future__ import annotations

import time

import numpy as np

from pathplanning.spaces.environment3d import Environment3D
from pathplanning.utils.sampling3d import is_collide, sample_free, steer


class Tree:
    """Simple tree structure used by RRT-Connect."""

    def __init__(self, node: tuple[float, ...]) -> None:
        """Initialize a tree with a root node.

        Args:
            node: Root state.
        """
        self.vertices: list[tuple[float, ...]] = []
        self.parent_by_node: dict[tuple[float, ...], tuple[float, ...]] = {}
        self.vertices.append(node)

    def add_vertex(self, node: tuple[float, ...]) -> None:
        """Insert a vertex if it does not already exist."""
        if node not in self.vertices:
            self.vertices.append(node)

    def add_edge(self, parent: tuple[float, ...], child: tuple[float, ...]) -> None:
        """Insert a directed edge parent -> child."""
        self.parent_by_node[child] = parent


class RrtConnect:
    """Bidirectional RRT-Connect planner in 3D."""

    def __init__(self, rng: np.random.Generator | None = None) -> None:
        """Initialize planner state and parameters."""
        self.env = Environment3D()
        self.parent_by_node: dict[tuple[float, ...], tuple[float, ...]] = {}
        self.vertices: list[tuple[float, ...]] = []
        self.edges: set[tuple[tuple[float, ...], tuple[float, ...]]] = set()
        self.i = 0
        self.maxiter = 10000
        self.stepsize = 0.5
        self.path_edges: list[tuple[tuple[float, ...], tuple[float, ...]]] = []
        self.done = False
        self.qinit = tuple(self.env.start)
        self.qgoal = tuple(self.env.goal)
        self.x0, self.xt = tuple(self.env.start), tuple(self.env.goal)
        self.qnew: tuple[float, ...] | None = None
        self.ind = 0
        self.rng = rng if rng is not None else np.random.default_rng()

    def build_rrt(self, qinit: tuple[float, ...]) -> Tree:
        """Build a single RRT tree from an initial state.

        Args:
            qinit: Root node for the tree.

        Returns:
            Constructed tree.
        """
        tree = Tree(qinit)
        for _ in range(self.maxiter):
            qrand = self.random_config()
            self.extend(tree, qrand)
        return tree

    def extend(self, tree: Tree, q: tuple[float, ...]) -> str:
        """Extend a tree toward target state `q`.

        Args:
            tree: Tree to be extended.
            q: Target state.

        Returns:
            One of `"Reached"`, `"Advanced"`, or `"Trapped"`.
        """
        qnear = tuple(self.nearest_neighbor(q, tree))
        qnew, _dist = steer(self, qnear, q)
        self.qnew = qnew
        if self.new_config(q, qnear, qnew, dist=None):
            tree.add_vertex(qnew)
            tree.add_edge(qnear, qnew)
            if qnew == q:
                return "Reached"
            return "Advanced"
        return "Trapped"

    def nearest_neighbor(self, q: tuple[float, ...], tree: Tree) -> tuple[float, ...]:
        """Find nearest tree vertex to a target state."""
        vertices = np.array(tree.vertices)
        if len(vertices) == 1:
            return tuple(vertices[0])
        xr = np.tile(q, (len(vertices), 1))
        dists = np.linalg.norm(xr - vertices, axis=1)
        return tuple(tree.vertices[int(np.argmin(dists))])

    def random_config(self) -> tuple[float, ...]:
        """Sample a random free configuration."""
        return tuple(sample_free(self, rng=self.rng))

    def new_config(
        self,
        _q: tuple[float, ...],
        qnear: tuple[float, ...],
        qnew: tuple[float, ...],
        dist: float | None = None,
    ) -> bool:
        """Validate a newly steered configuration.

        Args:
            _q: Unused target state.
            qnear: Nearest existing state.
            qnew: Newly proposed state.
            dist: Optional steering distance.

        Returns:
            `True` if the new edge is collision-free.
        """
        collide, _ = is_collide(self, qnear, qnew, dist=dist)
        return not collide

    def connect(self, tree: Tree, q: tuple[float, ...]) -> str:
        """Iteratively extend `tree` toward `q` until blocked or reached."""
        print("in connect")
        while True:
            status = self.extend(tree, q)
            if status != "Advanced":
                break
        return status

    def plan(self, qinit: tuple[float, ...], qgoal: tuple[float, ...]) -> str | None:
        """Run bidirectional RRT-Connect between start and goal.

        Args:
            qinit: Start state.
            qgoal: Goal state.

        Returns:
            `None` on success, `"Failure"` if no connection is found.
        """
        tree_a = Tree(qinit)
        tree_b = Tree(qgoal)
        for k in range(self.maxiter):
            print(k)
            qrand = self.random_config()
            if self.extend(tree_a, qrand) != "Trapped":
                qnew = self.qnew
                if qnew is None:
                    return "Failure"
                if self.connect(tree_b, qnew) == "Reached":
                    print("reached")
                    self.done = True
                    self.path_edges = self.path(tree_a, tree_b)
                    return None
            tree_a, tree_b = self.swap(tree_a, tree_b)
        return "Failure"

    @staticmethod
    def swap(tree_a: Tree, tree_b: Tree) -> tuple[Tree, Tree]:
        """Swap two trees."""
        return tree_b, tree_a

    def path(self, tree_a: Tree, tree_b: Tree) -> list[tuple[tuple[float, ...], tuple[float, ...]]]:
        """Reconstruct path by backtracking through both trees."""
        if self.qnew is None:
            return []
        qnew = self.qnew
        path_a: list[tuple[tuple[float, ...], tuple[float, ...]]] = []
        path_b: list[tuple[tuple[float, ...], tuple[float, ...]]] = []
        while True:
            path_a.append((tree_a.parent_by_node[qnew], qnew))
            qnew = tree_a.parent_by_node[qnew]
            if qnew == self.qinit or qnew == self.qgoal:
                break
        qnew = self.qnew
        while True:
            path_b.append((tree_b.parent_by_node[qnew], qnew))
            qnew = tree_b.parent_by_node[qnew]
            if qnew == self.qinit or qnew == self.qgoal:
                break
        return path_a + path_b


if __name__ == "__main__":
    planner = RrtConnect()
    start_time = time.time()
    planner.plan(planner.qinit, planner.qgoal)
    print("time used = " + str(time.time() - start_time))
