"""Dynamic RRT planner over continuous-space contracts."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
from typing import Any, Literal, Protocol, TypeAlias

import numpy as np
from numpy.typing import NDArray
from scipy.spatial import cKDTree

from examples.worlds.demo_3d_world import build_demo_3d_world
from pathplanning.core.contracts import ContinuousSpace, State

Node: TypeAlias = tuple[float, float, float]
Edge: TypeAlias = tuple[Node, Node]
PathEdge: TypeAlias = NDArray[np.float64]
Obstacle: TypeAlias = Sequence[float] | NDArray[np.float64]


@dataclass(frozen=True)
class DynamicRRT3DConfig:
    """Runtime configuration for :class:`DynamicRRT3D`."""

    step_size: float = 0.25
    max_iterations: int = 10_000
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
        self._nodes: list[Node] = []

    def reset(self, nodes: Sequence[Node]) -> None:
        self._nodes = list(nodes)

    def add(self, node: Node) -> None:
        self._nodes.append(node)

    def nearest(self, target: Node) -> Node:
        if not self._nodes:
            raise ValueError("nearest called with an empty node set")
        if len(self._nodes) == 1:
            return self._nodes[0]
        points = np.asarray(self._nodes, dtype=float)
        target_vec = np.asarray(target, dtype=float)
        distances = np.linalg.norm(points - target_vec, axis=1)
        return _as_node(points[int(np.argmin(distances))])


class KDTreeNearestNodeIndex:
    """Exact nearest-neighbor index with batched cKDTree rebuilds."""

    def __init__(self, rebuild_threshold: int = 64) -> None:
        self._rebuild_threshold = max(1, rebuild_threshold)
        self._indexed_points: NDArray[np.float64] = np.empty((0, 3), dtype=float)
        self._pending_nodes: list[Node] = []
        self._tree: cKDTree | None = None

    def reset(self, nodes: Sequence[Node]) -> None:
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
        self._pending_nodes.append(node)

    def _rebuild_if_needed(self) -> None:
        if self._tree is None and self._pending_nodes:
            points = np.asarray(self._pending_nodes, dtype=float)
            self._indexed_points = points
            self._tree = cKDTree(self._indexed_points)
            self._pending_nodes = []
            return

        if len(self._pending_nodes) >= self._rebuild_threshold:
            pending = np.asarray(self._pending_nodes, dtype=float)
            if self._indexed_points.size == 0:
                self._indexed_points = pending
            else:
                self._indexed_points = np.vstack([self._indexed_points, pending])
            self._tree = cKDTree(self._indexed_points)
            self._pending_nodes = []

    def nearest(self, target: Node) -> Node:
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
            pending = np.asarray(self._pending_nodes, dtype=float)
            target_vec = np.asarray(target, dtype=float)
            pending_distances = np.linalg.norm(pending - target_vec, axis=1)
            pending_index = int(np.argmin(pending_distances))
            pending_distance = float(pending_distances[pending_index])
            pending_candidate = _as_node(pending[pending_index])
            if pending_distance < best_distance:
                best_node = pending_candidate
                best_distance = pending_distance

        if best_node is None:
            raise ValueError("nearest called with an empty node set")
        return best_node


def _as_node(point: Sequence[float] | NDArray[Any]) -> Node:
    """Convert a point-like object to an immutable 3D node tuple."""
    return float(point[0]), float(point[1]), float(point[2])


def _as_state(point: Node) -> State:
    return np.asarray(point, dtype=float)


def get_dist(pos1: Node, pos2: Node) -> float:
    """Return Euclidean distance between two nodes."""
    p1 = np.asarray(pos1, dtype=float)
    p2 = np.asarray(pos2, dtype=float)
    return float(np.linalg.norm(p2 - p1))


def steer_node(initparams: Any, x: Node, y: Node) -> tuple[Node, float]:
    """Steer from ``x`` toward ``y`` with planner step-size limits."""
    extended = initparams.space.steer(_as_state(x), _as_state(y), initparams.stepsize)
    child = _as_node(extended)
    return child, get_dist(x, child)


class DynamicRRT3D:
    """Dynamic RRT planner that depends only on ``ContinuousSpace`` contracts."""

    def __init__(
        self,
        environment: ContinuousSpace[State] | None = None,
        *,
        config: DynamicRRT3DConfig | None = None,
        rng: np.random.Generator | None = None,
        nearest_index: NearestNodeIndex | None = None,
        start: Sequence[float] | State | None = None,
        goal: Sequence[float] | State | None = None,
    ) -> None:
        self.config = config if config is not None else DynamicRRT3DConfig()
        self.rng = rng if rng is not None else np.random.default_rng()

        if environment is None:
            default_space, default_start, default_goal = build_demo_3d_world()
            self.space: ContinuousSpace[State] = default_space
            start_state = default_start if start is None else start
            goal_state = default_goal if goal is None else goal
        else:
            self.space = environment
            start_state = np.asarray([2.0, 2.0, 2.0], dtype=float) if start is None else start
            goal_state = np.asarray([6.0, 16.0, 0.0], dtype=float) if goal is None else goal

        self.env = self.space
        self.x0: Node = _as_node(np.asarray(start_state, dtype=float))
        self.xt: Node = _as_node(np.asarray(goal_state, dtype=float))
        self.qrobot: Node = self.x0
        self.current: Node = self.x0

        self.stepsize = float(self.config.step_size)
        self.maxiter = int(self.config.max_iterations)
        self.goal_prob = float(self.config.goal_sample_probability)
        self.way_point_prob = float(self.config.way_point_sample_probability)

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
        environment: ContinuousSpace[State] | None = None,
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

    def init_rrt(self) -> None:
        """Initialize the tree with the root node."""
        self.nodes = [self.x0]
        self.parent_by_node = {}
        self.edges = set()
        self.node_state = {self.x0: "valid"}
        self._nearest_index.reset(self.nodes)

    def add_node(self, parent_node: Node, extended: Node) -> None:
        """Add a node and its parent-edge relation to the tree."""
        if extended in self.node_state:
            return
        self.nodes.append(extended)
        self.parent_by_node[extended] = parent_node
        self.edges.add((extended, parent_node))
        self.node_state[extended] = "valid"
        self._nearest_index.add(extended)

    def nearest(self, target: Node) -> Node:
        """Return nearest existing tree node to ``target``."""
        return self._nearest_index.nearest(target)

    def random_state(self) -> Node:
        """Generate a random collision-free state using planner RNG."""
        return _as_node(self.space.sample_free(self.rng))

    def choose_target(self) -> Node:
        """Sample target node with goal and waypoint bias."""
        p = float(self.rng.random())
        index = 0 if len(self.nodes) <= 1 else int(self.rng.integers(0, high=len(self.nodes) - 1))
        if p < self.goal_prob:
            return self.xt
        if self.nodes and p < self.goal_prob + self.way_point_prob:
            return self.nodes[index]
        return self.random_state()

    def extend(self, parent_node: Node, target: Node) -> tuple[Node, bool]:
        """Attempt one-step extension from ``parent_node`` toward ``target``."""
        extended, dist = steer_node(self, parent_node, target)
        _ = dist
        collide = not self.space.is_motion_valid(_as_state(parent_node), _as_state(extended))
        return extended, collide

    def grow_rrt(self) -> None:
        """Grow the RRT until the goal is reached or iteration limit is hit."""
        self.ind = 0
        while self.ind <= self.maxiter:
            target = self.choose_target()
            nearest = self.nearest(target)
            extended, collide = self.extend(nearest, target)
            if not collide:
                self.add_node(nearest, extended)
                if get_dist(extended, self.xt) <= self.stepsize:
                    goal_motion_valid = self.space.is_motion_valid(
                        _as_state(extended), _as_state(self.xt)
                    )
                    if goal_motion_valid:
                        self.add_node(extended, self.xt)
                        self.node_state[self.xt] = "valid"
                        self.done = True
                        break
                self.i += 1
            self.ind += 1

    def regrow_rrt(self) -> None:
        """Trim invalid subtrees and regrow the tree."""
        self.trim_rrt()
        self.grow_rrt()

    def trim_rrt(self) -> None:
        """Remove invalid nodes induced by dynamic obstacle updates."""
        kept: list[Node] = [self.x0]
        for node in self.nodes[1:]:
            parent = self.parent_by_node.get(node)
            if parent is None:
                continue
            if self.node_state.get(parent) == "invalid":
                self.node_state[node] = "invalid"
            if self.node_state.get(node) != "invalid":
                kept.append(node)
        self.create_tree_from_nodes(kept)

    def find_affected_edges(self, obstacle: Obstacle) -> list[Edge]:
        """Find edges that are no longer valid after an obstacle update."""
        _ = obstacle
        affected: list[Edge] = []
        for edge in self.edges:
            child, parent = edge
            collide = not self.space.is_motion_valid(_as_state(child), _as_state(parent))
            if collide:
                affected.append(edge)
        return affected

    def invalidate_nodes(self, obstacle: Obstacle) -> None:
        """Mark nodes invalid if their incoming edges now collide."""
        for edge in self.find_affected_edges(obstacle):
            self.node_state[self.child_endpoint_node(edge)] = "invalid"

    def child_endpoint_node(self, edge: Edge) -> Node:
        """Return child endpoint of an edge tuple ``(child, parent)``."""
        return edge[0]

    def create_tree_from_nodes(self, nodes: list[Node]) -> None:
        """Rebuild edge set from a filtered node list."""
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
        node = self.xt
        max_hops = max(1, len(self.nodes) + 1)
        for _ in range(max_hops):
            if node == self.x0:
                return path_edges, dist
            parent = self.parent_by_node.get(node)
            if parent is None:
                return None
            path_edges.append(np.array([node, parent], dtype=float))
            dist += get_dist(node, parent)
            node = parent
        return None

    def main(self) -> None:
        """Run a minimal dynamic-planning demonstration loop."""
        self.init_rrt()
        self.grow_rrt()
        path_result = self.path()
        if path_result is None:
            return
        self.path_segments, _ = path_result

    def visualization(self) -> None:
        """Deprecated no-op. Use plotting helpers from ``pathplanning.viz.rrt_3d``."""
        return None


__all__ = [
    "DynamicRRT3DConfig",
    "NearestNodeIndex",
    "BruteForceNearestNodeIndex",
    "KDTreeNearestNodeIndex",
    "DynamicRRT3D",
    "Node",
    "Edge",
    "PathEdge",
    "Obstacle",
]


if __name__ == "__main__":
    planner = DynamicRRT3D()
    planner.main()
