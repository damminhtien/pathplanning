"""Dynamic RRT planner for 3D environments."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
from typing import Any, Literal, Protocol, TypeAlias, cast

import numpy as np
from numpy.typing import NDArray
from scipy.spatial import cKDTree

from pathplanning.spaces.environment3d import Environment3D
from pathplanning.utils import sampling3d as utils_3d

Node: TypeAlias = tuple[float, float, float]
Edge: TypeAlias = tuple[Node, Node]
PathEdge: TypeAlias = NDArray[np.float64]
Obstacle: TypeAlias = Sequence[float] | NDArray[np.float64]


@dataclass(frozen=True)
class DynamicRRT3DConfig:
    """Runtime configuration for :class:`DynamicRRT3D`."""

    step_size: float = 0.25
    max_iterations: int = 10000
    goal_sample_probability: float = 0.05
    way_point_sample_probability: float = 0.02
    dynamic_step_limit: int = 8
    nearest_rebuild_threshold: int = 64


class NearestNodeIndex(Protocol):
    """Contract for nearest-neighbor implementations used by the planner."""

    def reset(self, nodes: Sequence[Node]) -> None:
        """Reset index state from the full current node set."""

    def add(self, node: Node) -> None:
        """Add one newly inserted node to the index."""

    def nearest(self, target: Node) -> Node:
        """Return nearest indexed node to ``target``."""


class BruteForceNearestNodeIndex:
    """Simple exact nearest-neighbor index based on vectorized NumPy distance."""

    def __init__(self) -> None:
        """Initialize an empty node container."""
        self._nodes: list[Node] = []

    def reset(self, nodes: Sequence[Node]) -> None:
        """Replace index content with ``nodes``."""
        self._nodes = list(nodes)

    def add(self, node: Node) -> None:
        """Append one node to the index."""
        self._nodes.append(node)

    def nearest(self, target: Node) -> Node:
        """Return the exact nearest node to ``target``."""
        if not self._nodes:
            raise ValueError("nearest called with an empty node set")
        if len(self._nodes) == 1:
            return self._nodes[0]
        node_matrix = np.asarray(self._nodes, dtype=float)
        target_repeated = np.tile(target, (len(node_matrix), 1))
        distances = np.linalg.norm(target_repeated - node_matrix, axis=1)
        return _as_node(node_matrix[int(np.argmin(distances))])


class KDTreeNearestNodeIndex:
    """Exact nearest-neighbor index with batched cKDTree rebuilds.

    New nodes are buffered and checked via brute force until the pending batch size
    reaches ``rebuild_threshold``. Then the tree is rebuilt, keeping exact results
    while reducing full rebuild frequency.
    """

    def __init__(self, rebuild_threshold: int = 64) -> None:
        """Initialize an empty KD-tree index.

        Args:
            rebuild_threshold: Number of pending points before rebuilding.
        """
        self._rebuild_threshold = max(1, rebuild_threshold)
        self._indexed_points: NDArray[np.float64] = np.empty((0, 3), dtype=float)
        self._pending_nodes: list[Node] = []
        self._tree: cKDTree | None = None

    def reset(self, nodes: Sequence[Node]) -> None:
        """Replace index content with ``nodes`` and rebuild internal tree."""
        points = np.asarray(nodes, dtype=float)
        if points.size == 0:
            self._indexed_points = np.empty((0, 3), dtype=float)
            self._tree = None
            self._pending_nodes = []
            return
        self._indexed_points = points
        self._tree = cKDTree(self._indexed_points)
        self._pending_nodes = []

    def add(self, node: Node) -> None:
        """Buffer one node insertion until next lazy rebuild."""
        self._pending_nodes.append(node)

    def _rebuild_if_needed(self) -> None:
        """Rebuild tree if pending inserts cross the configured threshold."""
        if self._tree is None and self._pending_nodes:
            points = np.asarray(self._pending_nodes, dtype=float)
            self._indexed_points = points
            self._tree = cKDTree(self._indexed_points)
            self._pending_nodes = []
            return

        if len(self._pending_nodes) >= self._rebuild_threshold:
            pending_points = np.asarray(self._pending_nodes, dtype=float)
            if self._indexed_points.size == 0:
                self._indexed_points = pending_points
            else:
                self._indexed_points = np.vstack([self._indexed_points, pending_points])
            self._tree = cKDTree(self._indexed_points)
            self._pending_nodes = []

    def nearest(self, target: Node) -> Node:
        """Return the exact nearest node to ``target``."""
        self._rebuild_if_needed()

        best_node: Node | None = None
        best_distance = np.inf

        if self._tree is not None and self._indexed_points.size > 0:
            _, index = self._tree.query(np.asarray(target, dtype=float), k=1)
            candidate = _as_node(self._indexed_points[int(index)])
            candidate_distance = get_dist(candidate, target)
            best_node = candidate
            best_distance = candidate_distance

        if self._pending_nodes:
            pending_points = np.asarray(self._pending_nodes, dtype=float)
            target_repeated = np.tile(target, (len(pending_points), 1))
            pending_distances = np.linalg.norm(target_repeated - pending_points, axis=1)
            pending_index = int(np.argmin(pending_distances))
            pending_distance = float(pending_distances[pending_index])
            pending_candidate = _as_node(pending_points[pending_index])
            if pending_distance < best_distance:
                best_node = pending_candidate
                best_distance = pending_distance

        if best_node is None:
            raise ValueError("nearest called with an empty node set")

        return best_node


def _as_node(point: Sequence[float] | NDArray[Any]) -> Node:
    """Convert a point-like object to an immutable 3D node tuple."""
    return float(point[0]), float(point[1]), float(point[2])


def get_dist(pos1: Node, pos2: Node) -> float:
    """Typed wrapper around ``utils_3d.get_dist``."""
    return float(utils_3d.get_dist(pos1, pos2))


def steer_node(initparams: Any, x: Node, y: Node) -> tuple[Node, float]:
    """Steer from ``x`` toward ``y`` with planner step-size limits."""
    child, dist = utils_3d.steer(initparams, x, y, DIST=True)
    return _as_node(cast(Sequence[float] | NDArray[Any], child)), float(dist)


def is_collide(
    initparams: Any,
    x: Node,
    child: Node,
    dist: float | None = None,
) -> tuple[bool, float]:
    """Typed wrapper around segment-obstacle collision checking."""
    collide, actual_dist = utils_3d.is_collide(initparams, x, child, dist)
    return bool(collide), float(actual_dist)


class DynamicRRT3D:
    """Dynamic RRT planner that regrows paths as obstacles move."""

    def __init__(
        self,
        environment: Environment3D | None = None,
        *,
        config: DynamicRRT3DConfig | None = None,
        rng: np.random.Generator | None = None,
        nearest_index: NearestNodeIndex | None = None,
    ) -> None:
        """Initialize planner state.

        Args:
            environment: Optional preconfigured environment instance.
            config: Optional runtime configuration.
            rng: Optional random number generator for deterministic sampling.
            nearest_index: Optional nearest-neighbor backend.
        """
        self.env = environment if environment is not None else Environment3D()
        self.config = config if config is not None else DynamicRRT3DConfig()
        self.rng = rng if rng is not None else np.random.default_rng()

        self.x0: Node = _as_node(self.env.start)
        self.xt: Node = _as_node(self.env.goal)
        self.qrobot: Node = self.x0
        self.current: Node = _as_node(self.env.start)

        self.stepsize = self.config.step_size
        self.maxiter = self.config.max_iterations
        self.goal_prob = self.config.goal_sample_probability
        self.way_point_prob = self.config.way_point_sample_probability

        self.done = False
        self.invalid = False

        self.nodes: list[Node] = []
        self.parent_by_node: dict[Node, Node] = {}
        self.edges: set[Edge] = set()
        self.path_segments: list[PathEdge] = []
        self.node_state: dict[Node, Literal["valid", "invalid"]] = {}
        self.ind = 0
        self.i = 0

        if nearest_index is not None:
            self._nearest_index = nearest_index
        else:
            self._nearest_index = KDTreeNearestNodeIndex(
                rebuild_threshold=self.config.nearest_rebuild_threshold
            )

    @classmethod
    def with_seed(
        cls,
        seed: int,
        environment: Environment3D | None = None,
        *,
        config: DynamicRRT3DConfig | None = None,
        nearest_index: NearestNodeIndex | None = None,
    ) -> DynamicRRT3D:
        """Create a planner with a deterministic seeded RNG."""
        return cls(
            environment=environment,
            config=config,
            rng=np.random.default_rng(seed),
            nearest_index=nearest_index,
        )

    def regrow_rrt(self) -> None:
        """Trim invalid subtrees and regrow the tree."""
        self.trim_rrt()
        self.grow_rrt()

    def trim_rrt(self) -> None:
        """Remove invalid nodes induced by dynamic obstacle updates."""
        nodes_to_keep: list[Node] = []
        i = 1
        print("trimming...")
        while i < len(self.nodes):
            qi = self.nodes[i]
            qp = self.parent_by_node[qi]
            if self.node_state.get(qp) == "invalid":
                self.node_state[qi] = "invalid"
            if self.node_state.get(qi) != "invalid":
                nodes_to_keep.append(qi)
            i += 1
        self.create_tree_from_nodes(nodes_to_keep)

    def invalidate_nodes(self, obstacle: Obstacle) -> None:
        """Mark nodes invalid if their incoming edges now collide."""
        edges = self.find_affected_edges(obstacle)
        for edge in edges:
            qe = self.child_endpoint_node(edge)
            self.node_state[qe] = "invalid"

    def init_rrt(self) -> None:
        """Initialize the tree with the root node."""
        self.nodes.append(self.x0)
        self.node_state[self.x0] = "valid"
        self._nearest_index.reset(self.nodes)

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
                    self.node_state[self.xt] = "valid"
                    break
                self.i += 1
            self.ind += 1

    def choose_target(self) -> Node:
        """Sample target node with goal and waypoint bias."""
        p = float(self.rng.random())
        if len(self.nodes) <= 1:
            i = 0
        else:
            i = int(self.rng.integers(0, high=len(self.nodes) - 1))
        if p < self.goal_prob:
            return self.xt
        if self.nodes and p < self.goal_prob + self.way_point_prob:
            return self.nodes[i]
        return self.random_state()

    def random_state(self) -> Node:
        """Generate a random collision-free state using planner RNG."""
        lower = np.asarray(self.env.boundary[0:3], dtype=float)
        upper = np.asarray(self.env.boundary[3:6], dtype=float)

        while True:
            xrand = self.rng.uniform(lower, upper)
            candidate = _as_node(xrand)
            if not utils_3d.is_inside(self, candidate):
                return candidate

    def add_node(self, parent_node: Node, extended: Node) -> None:
        """Add a node and its parent-edge relation to the tree."""
        self.nodes.append(extended)
        self.parent_by_node[extended] = parent_node
        self.edges.add((extended, parent_node))
        self.node_state[extended] = "valid"
        self._nearest_index.add(extended)

    def nearest(self, target: Node) -> Node:
        """Return nearest existing tree node to ``target``."""
        return self._nearest_index.nearest(target)

    def extend(self, parent_node: Node, target: Node) -> tuple[Node, bool]:
        """Attempt one-step extension from ``parent_node`` toward ``target``."""
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
        self.path_segments, _path_dist = path_result

        self.done = True
        t = 0
        while t < self.config.dynamic_step_limit:
            move_result = self.env.move_block(a=[0.2, 0, -0.2], mode="translation")
            if move_result is None:
                break
            new, _ = move_result
            self.invalidate_nodes(new)
            self.trim_rrt()

            self.invalid = self.path_is_invalid(self.path_segments)
            if self.invalid:
                self.done = False
                self.regrow_rrt()
                self.path_segments = []
                path_result = self.path()
                if path_result is None:
                    t += 1
                    continue
                self.path_segments, _path_dist = path_result
                self.done = True
            t += 1

        return None

    def find_affected_edges(self, obstacle: Obstacle) -> list[Edge]:
        """Find edges that collide after obstacle motion."""
        _ = obstacle
        print("finding affected edges...")
        affected_edges: list[Edge] = []
        for edge in self.edges:
            child, parent = edge
            collide, _ = is_collide(self, child, parent)
            if collide:
                affected_edges.append(edge)
        return affected_edges

    def child_endpoint_node(self, edge: Edge) -> Node:
        """Return child endpoint of an edge tuple ``(child, parent)``."""
        return edge[0]

    def create_tree_from_nodes(self, nodes: list[Node]) -> None:
        """Rebuild edge set from a filtered node list."""
        print("creating tree...")
        self.nodes = list(nodes)
        self.edges = {
            (node, self.parent_by_node[node]) for node in nodes if node in self.parent_by_node
        }
        self._nearest_index.reset(self.nodes)

    def path_is_invalid(self, path: list[PathEdge]) -> bool:
        """Check whether any path segment includes an invalid endpoint."""
        for edge in path:
            start, end = _as_node(edge[0]), _as_node(edge[1])
            if self.node_state.get(start) == "invalid" or self.node_state.get(end) == "invalid":
                return True
        return False

    def path(self, dist: float = 0.0) -> tuple[list[PathEdge], float] | None:
        """Backtrack from goal to start and build edge list with total distance."""
        path_edges: list[PathEdge] = []
        x = self.xt
        i = 0
        while x != self.x0:
            x2 = self.parent_by_node.get(x)
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
        """Deprecated no-op. Use plotting helpers from ``pathplanning.viz.rrt_3d``."""
        return None


if __name__ == "__main__":
    rrt = DynamicRRT3D()
    rrt.main()
