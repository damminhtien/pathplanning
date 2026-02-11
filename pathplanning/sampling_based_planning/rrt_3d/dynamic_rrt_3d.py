"""Dynamic RRT planner for 3D environments."""

from __future__ import annotations

from collections.abc import Sequence
from typing import Any, Literal, TypeAlias, cast

import numpy as np
from numpy.typing import NDArray
from pathplanning.viz import lazy_import

from . import plot_util_3d, utils_3d
from .env_3d import Environment3D

plt = lazy_import("matplotlib.pyplot")

Node: TypeAlias = tuple[float, float, float]
Edge: TypeAlias = tuple[Node, Node]
PathEdge: TypeAlias = NDArray[np.float64]
Obstacle: TypeAlias = Sequence[float] | NDArray[np.float64]


def _as_node(point: Sequence[float] | NDArray[Any]) -> Node:
    """Convert a point-like object to an immutable 3D node tuple."""
    return float(point[0]), float(point[1]), float(point[2])


def get_dist(pos1: Node, pos2: Node) -> float:
    """Typed wrapper around `utils_3d.getDist`."""
    return float(utils_3d.getDist(pos1, pos2))


def sample_free_state(initparams: Any, bias: float = 0.1) -> Node:
    """Sample a collision-free state and convert to the `Node` type."""
    sampled = cast(Sequence[float] | NDArray[Any], utils_3d.sampleFree(initparams, bias=bias))
    return _as_node(sampled)


def nearest_node(initparams: Any, target: Node) -> Node:
    """Return the nearest node in the current tree."""
    nearest = cast(Sequence[float] | NDArray[Any], utils_3d.nearest(initparams, target, isset=True))
    return _as_node(nearest)


def steer_node(initparams: Any, x: Node, y: Node) -> tuple[Node, float]:
    """Steer from `x` toward `y` with planner step-size limits."""
    child, dist = utils_3d.steer(initparams, x, y, DIST=True)
    return _as_node(cast(Sequence[float] | NDArray[Any], child)), float(dist)


def is_collide(
    initparams: Any,
    x: Node,
    child: Node,
    dist: float | None = None,
) -> tuple[bool, float]:
    """Typed wrapper around segment-obstacle collision checking."""
    collide, actual_dist = utils_3d.isCollide(initparams, x, child, dist)
    return bool(collide), float(actual_dist)


def set_axes_equal_typed(ax: Any) -> None:
    """Set equal aspect ratio for 3D axes."""
    plot_util_3d.set_axes_equal(ax)


def draw_block_list_typed(
    ax: Any,
    blocks: NDArray[np.float64],
    color: Any | None = None,
    alpha: float = 0.15,
) -> Any:
    """Typed wrapper for block rendering."""
    return plot_util_3d.draw_block_list(ax, blocks, color=color, alpha=alpha)


def draw_spheres_typed(ax: Any, balls: NDArray[np.float64]) -> None:
    """Typed wrapper for sphere rendering."""
    plot_util_3d.draw_Spheres(ax, balls)


def draw_obb_typed(ax: Any, obb_items: Any, color: Any | None = None, alpha: float = 0.15) -> Any:
    """Typed wrapper for OBB rendering."""
    return plot_util_3d.draw_obb(ax, obb_items, color=color, alpha=alpha)


def draw_line_typed(
    ax: Any,
    segments: NDArray[np.float64],
    visibility: float = 1.0,
    color: Any | None = None,
) -> None:
    """Typed wrapper for line rendering."""
    plot_util_3d.draw_line(ax, segments, visibility=visibility, color=color)


def make_transparent_typed(ax: Any) -> None:
    """Apply transparent panes/grid style to a 3D axis."""
    plot_util_3d.make_transparent(ax)


class DynamicRRT3D:
    """Dynamic RRT planner that regrows paths as obstacles move."""

    def __init__(self) -> None:
        """Initialize planner state and environment."""
        self.env = Environment3D()
        self.x0: Node = _as_node(self.env.start)
        self.xt: Node = _as_node(self.env.goal)
        self.qrobot: Node = self.x0
        self.current: Node = _as_node(self.env.start)
        self.stepsize = 0.25
        self.maxiter = 10000
        self.GoalProb = 0.05  # legacy attribute used by older modules
        self.WayPointProb = 0.02  # legacy attribute used by older modules
        self.done = False
        self.invalid = False

        # Legacy attribute names kept for compatibility with utility functions.
        self.V: list[Node] = []
        self.Parent: dict[Node, Node] = {}
        self.Edge: set[Edge] = set()
        self.Path: list[PathEdge] = []
        self.flag: dict[Node, Literal["Valid", "Invalid"]] = {}
        self.ind = 0
        self.i = 0

    def regrow_rrt(self) -> None:
        """Trim invalid subtrees and regrow the tree."""
        self.trim_rrt()
        self.grow_rrt()

    def trim_rrt(self) -> None:
        """Remove invalid nodes induced by dynamic obstacle updates."""
        nodes_to_keep: list[Node] = []
        i = 1
        print("trimming...")
        while i < len(self.V):
            qi = self.V[i]
            qp = self.Parent[qi]
            if self.flag.get(qp) == "Invalid":
                self.flag[qi] = "Invalid"
            if self.flag.get(qi) != "Invalid":
                nodes_to_keep.append(qi)
            i += 1
        self.create_tree_from_nodes(nodes_to_keep)

    def invalidate_nodes(self, obstacle: Obstacle) -> None:
        """Mark nodes invalid if their incoming edges now collide."""
        edges = self.find_affected_edges(obstacle)
        for edge in edges:
            qe = self.child_endpoint_node(edge)
            self.flag[qe] = "Invalid"

    def init_rrt(self) -> None:
        """Initialize the tree with the root node."""
        self.V.append(self.x0)
        self.flag[self.x0] = "Valid"

    def grow_rrt(self) -> None:
        """Grow the RRT until the goal is reached or iteration limit is hit."""
        print("growing...")
        qnew = self.x0
        distance_threshold = self.stepsize
        self.ind = 0
        while self.ind <= self.maxiter:
            qtarget = self.choose_target()
            qnearest = self.nearest(qtarget)
            qnew, collide = self.extend(qnearest, qtarget)
            if not collide:
                self.add_node(qnearest, qnew)
                if get_dist(qnew, self.xt) < distance_threshold:
                    self.add_node(qnearest, self.xt)
                    self.flag[self.xt] = "Valid"
                    break
                self.i += 1
            self.ind += 1

    def choose_target(self) -> Node:
        """Sample target node with goal and waypoint bias."""
        p = np.random.uniform()
        if len(self.V) <= 1:
            i = 0
        else:
            i = np.random.randint(0, high=len(self.V) - 1)
        if p < self.GoalProb:
            return self.xt
        if self.V and p < self.GoalProb + self.WayPointProb:
            return self.V[i]
        return self.random_state()

    def random_state(self) -> Node:
        """Generate a random collision-free state."""
        return sample_free_state(self, bias=0.0)

    def add_node(self, parent_node: Node, extended: Node) -> None:
        """Add a node and its parent-edge relation to the tree."""
        self.V.append(extended)
        self.Parent[extended] = parent_node
        self.Edge.add((extended, parent_node))
        self.flag[extended] = "Valid"

    def nearest(self, target: Node) -> Node:
        """Return nearest existing tree node to `target`."""
        return nearest_node(self, target)

    def extend(self, parent_node: Node, target: Node) -> tuple[Node, bool]:
        """Attempt one-step extension from `parent_node` toward `target`."""
        extended, dist = steer_node(self, parent_node, target)
        collide, _ = is_collide(self, parent_node, target, dist)
        return extended, collide

    def main(self) -> None:
        """Run the dynamic planning demonstration loop."""
        self.x0 = _as_node(self.env.goal)
        self.xt = _as_node(self.env.start)
        self.init_rrt()
        self.grow_rrt()

        path_result = self.path()
        if path_result is None:
            return
        self.Path, _path_dist = path_result

        self.done = True
        self.visualization()
        t = 0
        while True:
            move_result = self.env.move_block(a=[0.2, 0, -0.2], mode="translation")
            if move_result is None:
                break
            new, _ = move_result
            self.invalidate_nodes(new)
            self.trim_rrt()

            self.visualization()
            self.invalid = self.path_is_invalid(self.Path)
            if self.invalid:
                self.done = False
                self.regrow_rrt()
                self.Path = []
                path_result = self.path()
                if path_result is None:
                    continue
                self.Path, _path_dist = path_result
                self.done = True
                self.visualization()
            if t == 8:
                break
            t += 1

        self.visualization()
        plt.show()

    def find_affected_edges(self, obstacle: Obstacle) -> list[Edge]:
        """Find edges that collide after obstacle motion."""
        _ = obstacle
        print("finding affected edges...")
        affected_edges: list[Edge] = []
        for edge in self.Edge:
            child, parent = edge
            collide, _ = is_collide(self, child, parent)
            if collide:
                affected_edges.append(edge)
        return affected_edges

    def child_endpoint_node(self, edge: Edge) -> Node:
        """Return child endpoint of an edge tuple `(child, parent)`."""
        return edge[0]

    def create_tree_from_nodes(self, nodes: list[Node]) -> None:
        """Rebuild edge set from a filtered node list."""
        print("creating tree...")
        self.V = list(nodes)
        self.Edge = {(node, self.Parent[node]) for node in nodes if node in self.Parent}

    def path_is_invalid(self, path: list[PathEdge]) -> bool:
        """Check whether any path segment includes an invalid endpoint."""
        for edge in path:
            start, end = _as_node(edge[0]), _as_node(edge[1])
            if self.flag.get(start) == "Invalid" or self.flag.get(end) == "Invalid":
                return True
        return False

    def path(self, dist: float = 0.0) -> tuple[list[PathEdge], float] | None:
        """Backtrack from goal to start and build edge list with total distance."""
        path_edges: list[PathEdge] = []
        x = self.xt
        i = 0
        while x != self.x0:
            x2 = self.Parent.get(x)
            if x2 is None:
                print("Path is not found")
                return None
            path_edges.append(np.array([x, x2], dtype=float))
            dist += get_dist(x, x2)
            x = x2
            if i > 10000:
                print("Path is not found")
                return None
            i += 1
        return path_edges, dist

    def visualization(self) -> None:
        """Render current environment, tree edges, and solution path."""
        if self.ind % 100 != 0 and not self.done:
            return

        path = np.array(self.Path, dtype=float)
        start = np.asarray(self.env.start, dtype=float)
        goal = np.asarray(self.env.goal, dtype=float)
        edges = np.array([list(i) for i in self.Edge], dtype=float)

        ax = plt.subplot(111, projection="3d")
        ax.view_init(elev=90.0, azim=0.0)
        ax.clear()

        draw_spheres_typed(ax, np.asarray(self.env.balls, dtype=float))
        draw_block_list_typed(ax, np.asarray(self.env.blocks, dtype=float))
        obb_items = getattr(self.env, "OBB", None)
        if obb_items is not None:
            draw_obb_typed(ax, obb_items)
        draw_block_list_typed(ax, np.array([self.env.boundary], dtype=float), alpha=0.0)
        draw_line_typed(ax, edges, visibility=0.75, color="g")
        draw_line_typed(ax, path, color="r")

        ax.plot(start[0:1], start[1:2], start[2:], "go", markersize=7, markeredgecolor="k")
        ax.plot(goal[0:1], goal[1:2], goal[2:], "ro", markersize=7, markeredgecolor="k")

        set_axes_equal_typed(ax)
        make_transparent_typed(ax)
        ax.set_axis_off()
        plt.pause(0.0001)

if __name__ == "__main__":
    rrt = DynamicRRT3D()
    rrt.main()
