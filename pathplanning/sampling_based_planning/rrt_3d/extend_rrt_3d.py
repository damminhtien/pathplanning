"""Extended RRT implementation for 3D planning."""

from __future__ import annotations

import numpy as np

from .env_3d import Environment3D
from .utils_3d import get_dist, is_collide, nearest, path, sample_free, steer


class ExtendRrt:
    """Extended RRT planner with waypoint-biased target sampling."""

    def __init__(self, rng: np.random.Generator | None = None) -> None:
        """Initialize planner state and parameters."""
        self.env = Environment3D()
        self.x0, self.xt = tuple(self.env.start), tuple(self.env.goal)
        self.current = tuple(self.env.start)
        self.stepsize = 0.5
        self.maxiter = 10000
        self.goal_prob = 0.05
        self.way_point_prob = 0.05

        self.done = False
        self.vertices: list[tuple[float, ...]] = []
        self.parent_by_node: dict[tuple[float, ...], tuple[float, ...]] = {}
        self.path_edges: list[np.ndarray] = []
        self.ind = 0
        self.i = 0
        self.rng = rng if rng is not None else np.random.default_rng()

    def plan(
        self, env: Environment3D, initial: tuple[float, ...], goal: tuple[float, ...]
    ) -> None:
        """Run a basic extend-RRT planning loop.

        Args:
            env: Planning environment instance.
            initial: Initial state.
            goal: Goal state.
        """
        threshold = self.stepsize
        nearest_node = initial
        self.vertices.append(initial)
        rrt_tree = initial
        while self.ind <= self.maxiter:
            target = self.choose_target(goal)
            nearest_node = self.nearest_node(rrt_tree, target)
            extended, collide = self.extend_toward(env, nearest_node, target)
            if not collide:
                self.add_node(rrt_tree, nearest_node, extended)
                if get_dist(nearest_node, goal) <= threshold:
                    self.add_node(rrt_tree, nearest_node, self.xt)
                    break
                self.i += 1
            self.ind += 1

        self.done = True
        self.path_edges, _ = path(self)

    def nearest_node(
        self, _tree: tuple[float, ...], target: tuple[float, ...]
    ) -> tuple[float, ...]:
        """Return nearest node in the current tree."""
        return nearest(self, target, isset=True)

    def extend_toward(
        self,
        _env: Environment3D,
        nearest_state: tuple[float, ...],
        target: tuple[float, ...],
    ) -> tuple[tuple[float, ...], bool]:
        """Steer from nearest state toward target and collision-check edge."""
        extended, dist = steer(self, nearest_state, target, DIST=True)
        collide, _ = is_collide(self, nearest_state, target, dist)
        return extended, collide

    def add_node(
        self,
        _tree: tuple[float, ...],
        nearest_state: tuple[float, ...],
        extended: tuple[float, ...],
    ) -> None:
        """Add a node and parent relation to the planner tree."""
        self.vertices.append(extended)
        self.parent_by_node[extended] = nearest_state

    def random_state(self) -> np.ndarray | tuple[float, ...]:
        """Sample a random collision-free state."""
        return sample_free(self, bias=0, rng=self.rng)

    def choose_target(self, _state: tuple[float, ...]) -> tuple[float, ...]:
        """Choose goal, waypoint, or random state using configured probabilities."""
        p = self.rng.uniform()
        if len(self.vertices) == 1:
            index = 0
        else:
            index = int(self.rng.integers(0, high=len(self.vertices) - 1))
        if 0 < p < self.goal_prob:
            return self.xt
        if self.goal_prob < p < self.goal_prob + self.way_point_prob:
            return self.vertices[index]
        return tuple(self.random_state())


if __name__ == "__main__":
    planner = ExtendRrt()
    planner.plan(planner.env, planner.x0, planner.xt)
