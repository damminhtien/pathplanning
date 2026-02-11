"""Utility helpers shared across legacy 3D sampling-based planners."""

from __future__ import annotations

from collections.abc import Sequence
from typing import Any

import numpy as np


def get_ray(x, y):
    """Return ray origin and direction from point ``x`` to point ``y``."""
    direc = [y[0] - x[0], y[1] - x[1], y[2] - x[2]]
    return np.array([x, direc])


def get_aabb(blocks: Sequence[Sequence[float]]) -> list[np.ndarray]:
    """Build axis-aligned bounding boxes from raw block coordinates."""
    aabb_list: list[np.ndarray] = []
    for block in blocks:
        aabb_list.append(np.array([np.add(block[0:3], -0.0), np.add(block[3:6], 0.0)]))
    return aabb_list


def get_dist(pos1: Sequence[float], pos2: Sequence[float]) -> float:
    """Compute Euclidean distance between two 3D positions."""
    return np.sqrt(
        sum([(pos1[0] - pos2[0]) ** 2, (pos1[1] - pos2[1])
            ** 2, (pos1[2] - pos2[2]) ** 2])
    )


# NOTE:
# Most helpers below operate on legacy planner objects that expose attributes
# such as ``env``, ``V``, ``E``, ``i``, ``maxiter``, and ``stepsize``.


def visualization(initparams: Any) -> None:
    """Lazy wrapper around plot visualization to avoid import-time side effects."""
    from .plot_util_3d import visualization as _visualization

    return _visualization(initparams)


def sample_free(
    initparams: Any,
    bias: float = 0.1,
    max_tries: int = 1000,
    rng: np.random.Generator | None = None,
) -> np.ndarray:
    """Sample collision-free states with bounded rejection sampling."""
    rng = rng if rng is not None else np.random.default_rng()

    if rng.random() < bias:
        return np.array(initparams.xt)

    last_sample = None
    for _ in range(max_tries):
        x = rng.uniform(initparams.env.boundary[0:3], initparams.env.boundary[3:6])
        last_sample = x
        if not is_inside(initparams, x):
            return x

    # Fallback to a uniform sample even if it collides.
    if last_sample is None:
        return rng.uniform(initparams.env.boundary[0:3], initparams.env.boundary[3:6])
    return last_sample


# ---------------------- Collision checking algorithms
def is_inside(initparams: Any, x: Sequence[float]) -> bool:
    """Return ``True`` when a state lies inside any obstacle."""
    for i in initparams.env.blocks:
        if is_in_bound(i, x):
            return True
    for i in initparams.env.obb:
        if is_in_bound(i, x, mode="obb"):
            return True
    for i in initparams.env.balls:
        if is_in_ball(i, x):
            return True
    return False


def is_in_bound(
    i: Any,
    x: Any,
    mode: str | bool = False,
    factor: float = 0.0,
    isarray: bool = False,
) -> Any:
    """Check inclusion in an AABB/OBB, optionally vectorized for arrays."""
    if mode == "obb":
        return is_in_obb(i, x, isarray)
    if isarray:
        compx = (i[0] - factor <= x[:, 0]) & (x[:, 0] < i[3] + factor)
        compy = (i[1] - factor <= x[:, 1]) & (x[:, 1] < i[4] + factor)
        compz = (i[2] - factor <= x[:, 2]) & (x[:, 2] < i[5] + factor)
        return compx & compy & compz
    else:
        return (
            i[0] - factor <= x[0] < i[3] + factor
            and i[1] - factor <= x[1] < i[4] + factor
            and i[2] - factor <= x[2] < i[5]
        )


def is_in_obb(i: Any, x: Any, isarray: bool = False) -> Any:
    """Check point inclusion against an oriented bounding box."""
    # transform the point from {W} to {body}
    if isarray:
        pts = (i.transform @ np.column_stack((x, np.ones(len(x)))).T).T[:, 0:3]
        block = [
            -i.extents[0],
            -i.extents[1],
            -i.extents[2],
            +i.extents[0],
            +i.extents[1],
            +i.extents[2],
        ]
        return is_in_bound(block, pts, isarray=isarray)
    pt = i.transform @ np.append(x, 1)
    block = [
        -i.extents[0],
        -i.extents[1],
        -i.extents[2],
        +i.extents[0],
        +i.extents[1],
        +i.extents[2],
    ]
    return is_in_bound(block, pt)


def is_in_ball(i: Sequence[float], x: Sequence[float], factor: float = 0.0) -> bool:
    """Check whether ``x`` lies inside or on a sphere obstacle."""
    if get_dist(i[0:3], x) <= i[3] + factor:
        return True
    return False


def line_sphere(
    p0: Sequence[float], p1: Sequence[float], ball: Sequence[float]
) -> bool:
    """Check whether a line segment intersects a sphere obstacle."""
    # https://cseweb.ucsd.edu/classes/sp19/cse291-d/Files/CSE291_13_CollisionDetection.pdf
    c, r = ball[0:3], ball[-1]
    line = [p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]]
    d1 = [c[0] - p0[0], c[1] - p0[1], c[2] - p0[2]]
    t = (1 / (line[0] * line[0] + line[1] * line[1] + line[2] * line[2])) * (
        line[0] * d1[0] + line[1] * d1[1] + line[2] * d1[2]
    )
    if t <= 0:
        if (d1[0] * d1[0] + d1[1] * d1[1] + d1[2] * d1[2]) <= r**2:
            return True
    elif t >= 1:
        d2 = [c[0] - p1[0], c[1] - p1[1], c[2] - p1[2]]
        if (d2[0] * d2[0] + d2[1] * d2[1] + d2[2] * d2[2]) <= r**2:
            return True
    elif 0 < t < 1:
        x = [p0[0] + t * line[0], p0[1] + t * line[1], p0[2] + t * line[2]]
        k = [c[0] - x[0], c[1] - x[1], c[2] - x[2]]
        if (k[0] * k[0] + k[1] * k[1] + k[2] * k[2]) <= r**2:
            return True
    return False


def line_aabb(
    p0: Sequence[float], p1: Sequence[float], dist: float, aabb: Any
) -> bool:
    """Check whether a line segment intersects an axis-aligned bounding box."""
    # https://www.gamasutra.com/view/feature/131790/simple_intersection_tests_for_games.php?print=1
    # aabb should have the attributes of P, E as center point and extents
    mid = [(p0[0] + p1[0]) / 2, (p0[1] + p1[1]) /
           2, (p0[2] + p1[2]) / 2]  # mid point
    I = [(p1[0] - p0[0]) / dist, (p1[1] - p0[1]) / dist,
         (p1[2] - p0[2]) / dist]  # unit direction
    hl = dist / 2  # radius
    t = [aabb.center[0] - mid[0], aabb.center[1] - mid[1], aabb.center[2] - mid[2]]
    # do any of the principal axis form a separting axis?
    if abs(t[0]) > (aabb.extents[0] + hl * abs(I[0])):
        return False
    if abs(t[1]) > (aabb.extents[1] + hl * abs(I[1])):
        return False
    if abs(t[2]) > (aabb.extents[2] + hl * abs(I[2])):
        return False
    # I.cross(s axis) ?
    r = aabb.extents[1] * abs(I[2]) + aabb.extents[2] * abs(I[1])
    if abs(t[1] * I[2] - t[2] * I[1]) > r:
        return False
    # I.cross(y axis) ?
    r = aabb.extents[0] * abs(I[2]) + aabb.extents[2] * abs(I[0])
    if abs(t[2] * I[0] - t[0] * I[2]) > r:
        return False
    # I.cross(z axis) ?
    r = aabb.extents[0] * abs(I[1]) + aabb.extents[1] * abs(I[0])
    if abs(t[0] * I[1] - t[1] * I[0]) > r:
        return False

    return True


def line_obb(
    p0: Sequence[float], p1: Sequence[float], dist: float, obb: Any
) -> bool:
    """Check whether a line segment intersects an oriented bounding box."""
    # transform points to obb frame
    res = obb.transform @ np.column_stack([np.array([p0, p1]), [1, 1]]).T
    # record old position and set the position to origin
    old_center, obb.center = obb.center, [0, 0, 0]
    # calculate segment-AABB testing
    ans = line_aabb(res[0:3, 0], res[0:3, 1], dist, obb)
    # reset the position
    obb.center = old_center
    return ans


def is_collide(
    initparams: Any,
    x: Sequence[float],
    child: Sequence[float],
    dist: float | None = None,
) -> tuple[bool, float]:
    """Check whether segment ``x -> child`` collides with obstacles."""
    if dist is None:
        dist = get_dist(x, child)
    # check in bound
    if not is_in_bound(initparams.env.boundary, child):
        return True, dist
    # check collision in AABB
    for i in range(len(initparams.env.aabb)):
        if line_aabb(x, child, dist, initparams.env.aabb[i]):
            return True, dist
    # check collision in ball
    for i in initparams.env.balls:
        if line_sphere(x, child, i):
            return True, dist
    # check collision with obb
    for i in initparams.env.obb:
        if line_obb(x, child, dist, i):
            return True, dist
    return False, dist


# ---------------------- leaf node extending algorithms
def nearest(initparams: Any, x: Sequence[float], isset: bool = False) -> tuple[float, ...]:
    """Return nearest vertex in ``initparams.vertices`` to target ``x``."""
    vertices_array = np.array(initparams.vertices)
    if initparams.i == 0:
        return initparams.vertices[0]
    tiled_target = np.tile(x, (len(vertices_array), 1))
    dists = np.linalg.norm(tiled_target - vertices_array, axis=1)
    return tuple(initparams.vertices[np.argmin(dists)])


def near(initparams: Any, x: Sequence[float]) -> np.ndarray:
    """Return nearby vertices used by RRT* rewiring."""
    # s = np.array(s)
    vertices_array = np.array(initparams.vertices)
    if initparams.i == 0:
        return [initparams.vertices[0]]
    vertex_count = len(initparams.vertices)
    eta = initparams.eta
    gamma = initparams.gamma
    # min{γRRT∗ (log(card (V ))/ card (V ))1/d, η}
    radius = min(gamma * ((np.log(vertex_count) / vertex_count) ** (1 / 3)), eta)
    if initparams.done:
        radius = 1
    tiled_target = np.tile(x, (len(vertices_array), 1))
    inside = np.linalg.norm(tiled_target - vertices_array, axis=1) < radius
    near_points = vertices_array[inside]
    return np.array(near_points)


def steer(
    initparams: Any,
    x: Sequence[float],
    y: Sequence[float],
    DIST: bool = False,
) -> tuple[tuple[float, float, float], float]:
    """Steer one step from ``x`` toward ``y``."""
    if np.equal(x, y).all():
        return x, 0.0
    dist, step = get_dist(y, x), initparams.stepsize
    step = min(dist, step)
    increment = (
        (y[0] - x[0]) / dist * step,
        (y[1] - x[1]) / dist * step,
        (y[2] - x[2]) / dist * step,
    )
    xnew = (x[0] + increment[0], x[1] + increment[1], x[2] + increment[2])
    # direc = (y - s) / np.linalg.norm(y - s)
    # xnew = s + initparams.stepsize * direc
    if DIST:
        return xnew, dist
    return xnew, dist


def cost(initparams: Any, x: tuple[float, ...]) -> float:
    """here use the additive recursive cost function"""
    if x == initparams.x0:
        return 0
    return cost(initparams, initparams.parent_by_node[x]) + get_dist(x, initparams.parent_by_node[x])


def cost_from_set(initparams: Any, x: tuple[float, ...]) -> float:
    """here use a incremental cost set function"""
    if x == initparams.x0:
        return 0
    return initparams.cost_cache[initparams.parent_by_node[x]] + get_dist(
        x, initparams.parent_by_node[x]
    )


def path(
    initparams: Any,
    path_edges: list[np.ndarray] | None = None,
    dist: float = 0.0,
) -> tuple[list[np.ndarray], float]:
    """Backtrack from goal to start and return path edges with total distance."""
    if path_edges is None:
        path_edges = []
    x = initparams.xt
    while x != initparams.x0:
        x2 = initparams.parent_by_node[x]
        path_edges.append(np.array([x, x2]))
        dist += get_dist(x, x2)
        x = x2
    return path_edges, dist


class EdgeSet:
    """Adjacency map storing directed edges by parent node."""

    def __init__(self) -> None:
        """Initialize an empty parent-to-children edge map."""
        self.edges_by_parent: dict[tuple[float, ...], set[tuple[float, ...]]] = {}

    def add_edge(self, edge: tuple[tuple[float, ...], tuple[float, ...]]) -> None:
        """Insert one directed edge ``(parent, child)``."""
        x, y = edge[0], edge[1]
        if x in self.edges_by_parent:
            self.edges_by_parent[x].add(y)
        else:
            self.edges_by_parent[x] = {y}

    def remove_edge(self, edge: tuple[tuple[float, ...], tuple[float, ...]]) -> None:
        """Remove one directed edge ``(parent, child)``."""
        x, y = edge[0], edge[1]
        self.edges_by_parent[x].remove(y)

    def get_edge(
        self, nodes: Sequence[Sequence[float]] | None = None
    ) -> list[tuple[tuple[float, ...], tuple[float, ...]]]:
        """Return edge list, optionally filtered to a subset of parent nodes."""
        edges: list[tuple[tuple[float, ...], tuple[float, ...]]] = []
        if nodes is None:
            for v in self.edges_by_parent:
                for n in self.edges_by_parent[v]:
                    edges.append((v, n))
        else:
            for v in nodes:
                for n in self.edges_by_parent[tuple(v)]:
                    edges.append((tuple(v), n))
        return edges

    def is_end_node(self, node: tuple[float, ...]) -> bool:
        """Return ``True`` when a node has no outgoing edge."""
        return node not in self.edges_by_parent


# ------------------------ use a linked list to express the tree
class TreeNode:
    """Node for linked-tree demonstrations in this module."""

    def __init__(self, data: tuple[float, ...]) -> None:
        """Create a tree node with no parent and empty children set."""
        self.pos = data
        self.parent: TreeNode | None = None
        self.child: set[TreeNode] = set()


def tree_add_edge(node_in_tree: TreeNode, x: tuple[float, ...]) -> TreeNode:
    """Attach a new child node to ``node_in_tree``."""
    node_to_add = TreeNode(x)
    # node_in_tree = tree_bfs(head, xparent)
    node_in_tree.child.add(node_to_add)
    node_to_add.parent = node_in_tree
    return node_to_add


def tree_bfs(head: TreeNode, x: tuple[float, ...]) -> TreeNode | None:
    """Breadth-first search for node state ``x``."""
    node = head
    queue = []
    queue.append(node)
    while queue:
        curr = queue.pop()
        if curr.pos == x:
            return curr
        for child_node in curr.child:
            queue.append(child_node)


def tree_nearest(head: TreeNode, x: tuple[float, ...]) -> TreeNode | None:
    """Return nearest tree node to ``x`` by brute-force BFS traversal."""
    min_dist = np.inf
    min_node = None

    queue = []
    queue.append(head)
    while queue:
        curr = queue.pop()
        dist = get_dist(curr.pos, x)
        # record the current best
        if dist < min_dist:
            min_dist, min_node = dist, curr
        # bfs
        for child_node in curr.child:
            queue.append(child_node)
    return min_node


def tree_steer(initparams: Any, node: TreeNode, x: tuple[float, ...]) -> tuple[float, float, float]:
    """Steer one step from ``x`` toward ``node.pos``."""
    dist, step = get_dist(node.pos, x), initparams.stepsize
    increment = (
        (node.pos[0] - x[0]) / dist * step,
        (node.pos[1] - x[1]) / dist * step,
        (node.pos[2] - x[2]) / dist * step,
    )
    xnew = (x[0] + increment[0], x[1] + increment[1], x[2] + increment[2])
    return xnew


def tree_print(
    head: TreeNode,
) -> tuple[list[tuple[float, ...]], list[list[tuple[float, ...]]]]:
    """Collect tree vertices and edges from a linked-node structure."""
    queue = []
    queue.append(head)
    verts = []
    edge = []
    while queue:
        curr = queue.pop()
        # print(curr.pos)
        verts.append(curr.pos)
        if curr.parent is None:
            pass
        else:
            edge.append([curr.pos, curr.parent.pos])
        for child in curr.child:
            queue.append(child)
    return verts, edge


def tree_path(initparams: Any, end_node: TreeNode) -> list[list[tuple[float, ...]]]:
    """Reconstruct a path from a tree leaf back to the root."""
    path_edges: list[list[tuple[float, ...]]] = []
    curr = end_node
    while curr.pos != initparams.x0:
        if curr.parent is None:
            break
        path_edges.append([curr.pos, curr.parent.pos])
        curr = curr.parent
    return path_edges


# ---------------KD tree, used for nearest neighbor search
class KdTree:
    """Distance helpers for periodic and antipodal 3D spaces."""

    def __init__(self) -> None:
        """Initialize a stateless distance helper."""

    def r1_dist(self, q: float, p: float) -> float:
        """Compute distance on the real line."""
        return abs(q - p)

    def s1_dist(self, q: float, p: float) -> float:
        """Compute wrap-around distance on S1."""
        return min(abs(q - p), 1 - abs(q - p))

    def p3_dist(self, q: Sequence[float], p: Sequence[float]) -> float:
        """Compute minimum antipodal cube distance in P3 space."""
        # cubes with antipodal points
        q1, q2, q3 = q
        p1, p2, p3 = p
        d1 = np.sqrt((q1 - p1) ** 2 + (q2 - p2) ** 2 + (q3 - p3) ** 2)
        d2 = np.sqrt((1 - abs(q1 - p1)) ** 2 + (1 - abs(q2 - p2))
                     ** 2 + (1 - abs(q3 - p3)) ** 2)
        d3 = np.sqrt((-q1 - p1) ** 2 + (-q2 - p2) ** 2 + (q3 + 1 - p3) ** 2)
        d4 = np.sqrt((-q1 - p1) ** 2 + (-q2 - p2) ** 2 + (q3 - 1 - p3) ** 2)
        d5 = np.sqrt((-q1 - p1) ** 2 + (q2 + 1 - p2) ** 2 + (-q3 - p3) ** 2)
        d6 = np.sqrt((-q1 - p1) ** 2 + (q2 - 1 - p2) ** 2 + (-q3 - p3) ** 2)
        d7 = np.sqrt((q1 + 1 - p1) ** 2 + (-q2 - p2) ** 2 + (-q3 - p3) ** 2)
        d8 = np.sqrt((q1 - 1 - p1) ** 2 + (-q2 - p2) ** 2 + (-q3 - p3) ** 2)
        return min(d1, d2, d3, d4, d5, d6, d7, d8)


if __name__ == "__main__":
    from .env_3d import Environment3D
    import time
    import matplotlib.pyplot as plt

    class RrtDemo:
        """Minimal demo for linked-tree utilities."""

        def __init__(self):
            self.env = Environment3D()
            self.x0, self.xt = tuple(self.env.start), tuple(self.env.goal)
            self.stepsize = 0.5
            self.maxiter = 10000
            self.ind, self.i = 0, 0
            self.done = False
            self.path_edges = []
            self.vertices = []
            self.parent_by_node: dict[tuple[float, ...], tuple[float, ...]] = {}

            self.head = TreeNode(self.x0)

        def run(self):
            while self.ind < self.maxiter:
                xrand = sample_free(self)  # O(1)
                nearest_node = tree_nearest(self.head, xrand)  # O(N)
                xnew = tree_steer(self, nearest_node, xrand)  # O(1)
                collide, _ = is_collide(
                    self, nearest_node.pos, xnew)  # O(num obs)
                if not collide:
                    new_node = tree_add_edge(nearest_node, xnew)  # O(1)
                    # if the path is found
                    if get_dist(xnew, self.xt) <= self.stepsize:
                        end_node = tree_add_edge(new_node, self.xt)
                        self.path_edges = tree_path(self, end_node)
                        break
                    self.i += 1
                self.ind += 1

            self.done = True
            self.vertices, edges = tree_print(self.head)
            self.parent_by_node = {child: parent for child, parent in edges}
            print(edges)
            visualization(self)
            plt.show()

    demo = RrtDemo()
    st = time.time()
    demo.run()
    print(time.time() - st)
