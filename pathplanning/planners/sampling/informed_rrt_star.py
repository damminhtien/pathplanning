"""Informed RRT* implementation for 3D planning.

References:
    J. D. Gammell, S. S. Srinivasa, and T. D. Barfoot, "Informed RRT*:
    Optimal sampling-based path planning focused via direct sampling of an
    admissible ellipsoidal heuristic," IROS, 2014.
"""

from __future__ import annotations

from typing import Any

import numpy as np

from pathplanning.spaces.environment3d import Environment3D
from pathplanning.utils.sampling3d import (
    get_dist,
    is_collide,
    is_inside,
    near,
    nearest,
    path,
    sample_free,
    steer,
)


def create_unit_sphere(radius: float = 1.0) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Create a sampled unit sphere mesh in Cartesian coordinates.

    Args:
        radius: Sphere radius.

    Returns:
        Tuple of `(x, y, z)` sampled mesh arrays.
    """
    phi = np.linspace(0, 2 * np.pi, 256).reshape(256, 1)
    theta = np.linspace(0, np.pi, 256).reshape(-1, 256)

    x = radius * np.sin(theta) * np.cos(phi)
    y = radius * np.sin(theta) * np.sin(phi)
    z = radius * np.cos(theta)
    return x, y, z


def draw_ellipsoid(ax: Any, rotation: np.ndarray, scale: np.ndarray, center: np.ndarray) -> None:
    """Draw an ellipsoid in world coordinates.

    Args:
        ax: Matplotlib 3D axes.
        rotation: 3x3 world-frame rotation matrix.
        scale: 3x3 scaling matrix.
        center: Ellipsoid center in world coordinates.
    """
    xs, ys, zs = create_unit_sphere()
    points = np.array([xs, ys, zs])
    world_points = rotation @ scale @ points + center
    ax.plot_surface(world_points[0], world_points[1], world_points[2], alpha=0.05, color="g")


class InformedRrtStar:
    """Informed RRT* planner in 3D."""

    def __init__(self, show_ellipse: bool = False, rng: np.random.Generator | None = None) -> None:
        """Initialize planner state.

        Args:
            show_ellipse: Whether to visualize the informed ellipsoid.
        """
        self.env = Environment3D()
        self.xstart, self.xgoal = tuple(self.env.start), tuple(self.env.goal)
        self.x0, self.xt = tuple(self.env.start), tuple(self.env.goal)
        self.parent_by_node: dict[tuple[float, ...], tuple[float, ...]] = {}
        self.path_edges: list[Any] = []
        self.max_iterations = 10000
        self.ind = 0
        self.i = 0

        self.stepsize = 1
        self.gamma = 500
        self.eta = self.stepsize
        self.rgoal = self.stepsize
        self.done = False

        self.rotation_matrix_world = np.zeros((3, 3))
        self.ellipse_scale = np.zeros((3, 3))
        self.xcenter = np.zeros(3)
        self.show_ellipse = show_ellipse
        self.rng = rng if rng is not None else np.random.default_rng()

    def informed_rrt(
        self,
    ) -> tuple[list[tuple[float, ...]], set[tuple[tuple[float, ...], tuple[float, ...]]]]:
        """Run Informed RRT* and return the tree.

        Returns:
            Tuple `(V, E)` with sampled vertices and edges.
        """
        self.vertices = [self.xstart]
        self.edges: set[tuple[tuple[float, ...], tuple[float, ...]]] = set()
        self.solution_nodes: set[tuple[float, ...]] = set()
        self.tree = (self.vertices, self.edges)

        edge_cost_weight = 1
        while self.ind <= self.max_iterations:
            print(self.ind)
            cbest = (
                np.inf
                if len(self.solution_nodes) == 0
                else min({self.cost(x_solution) for x_solution in self.solution_nodes})
            )

            xrand = self.sample(self.xstart, self.xgoal, cbest)
            xnearest = nearest(self, xrand)
            xnew, dist = steer(self, xnearest, xrand)
            collide, _ = is_collide(self, xnearest, xnew, dist=dist)
            if not collide:
                self.vertices.append(xnew)
                x_near_set = near(self, xnew)
                xmin = xnearest
                cmin = self.cost(xmin) + edge_cost_weight * self.line(xnearest, xnew)
                for xnear in x_near_set:
                    xnear = tuple(xnear)
                    cnew = self.cost(xnear) + edge_cost_weight * self.line(xnear, xnew)
                    if cnew < cmin:
                        collide, _ = is_collide(self, xnear, xnew)
                        if not collide:
                            xmin = xnear
                            cmin = cnew
                self.edges.add((xmin, xnew))
                self.parent_by_node[xnew] = xmin

                for xnear in x_near_set:
                    xnear = tuple(xnear)
                    cnear = self.cost(xnear)
                    cnew = self.cost(xnew) + edge_cost_weight * self.line(xnew, xnear)
                    if cnew < cnear:
                        collide, _ = is_collide(self, xnew, xnear)
                        if not collide:
                            xparent = self.parent_by_node[xnear]
                            self.edges.difference_update((xparent, xnear))
                            self.edges.add((xnew, xnear))
                            self.parent_by_node[xnear] = xnew
                self.i += 1
                if self.in_goal_region(xnew):
                    print("reached")
                    self.done = True
                    self.parent_by_node[self.xgoal] = xnew
                    self.path_edges, _ = path(self)
                    self.solution_nodes.add(xnew)
            if self.done:
                self.path_edges, _ = path(self, path_edges=[])
            self.ind += 1
        return self.tree

    def sample(
        self,
        xstart: tuple[float, ...],
        xgoal: tuple[float, ...],
        cmax: float,
        bias: float = 0.05,
    ) -> np.ndarray | tuple[float, ...]:
        """Sample either informed ellipsoid or free space.

        Args:
            xstart: Start state.
            xgoal: Goal state.
            cmax: Current best path cost.
            bias: Goal sampling bias used by fallback free sampling.

        Returns:
            Sampled state.
        """
        if cmax < np.inf:
            cmin = get_dist(xgoal, xstart)
            xcenter = np.array(
                [
                    (xgoal[0] + xstart[0]) / 2,
                    (xgoal[1] + xstart[1]) / 2,
                    (xgoal[2] + xstart[2]) / 2,
                ]
            )
            rotation = self.rotation_to_world_frame(xstart, xgoal)
            radius = np.zeros(3)
            radius[0] = cmax / 2
            for i in range(1, 3):
                radius[i] = np.sqrt(cmax**2 - cmin**2) / 2
            scale = np.diag(radius)
            xball = self.sample_unit_ball()
            xrand = rotation @ scale @ xball + xcenter
            self.rotation_matrix_world = rotation
            self.xcenter = xcenter
            self.ellipse_scale = scale
            if is_inside(self, xrand):
                return self.sample(xstart, xgoal, cmax)
            return xrand
        return sample_free(self, bias=bias, rng=self.rng)

    def sample_unit_ball(self) -> np.ndarray:
        """Sample a point in a 3D unit ball (spherical coordinates)."""
        radius = self.rng.uniform(0.0, 1.0)
        theta = self.rng.uniform(0, np.pi)
        phi = self.rng.uniform(0, 2 * np.pi)
        x = radius * np.sin(theta) * np.cos(phi)
        y = radius * np.sin(theta) * np.sin(phi)
        z = radius * np.cos(theta)
        return np.array([x, y, z])

    def rotation_to_world_frame(
        self, xstart: tuple[float, ...], xgoal: tuple[float, ...]
    ) -> np.ndarray:
        """Compute rotation that aligns x-axis with start-to-goal direction."""
        distance = get_dist(xstart, xgoal)
        xstart_arr, xgoal_arr = np.array(xstart), np.array(xgoal)
        a1 = (xgoal_arr - xstart_arr) / distance
        matrix = np.outer(a1, [1, 0, 0])
        u, _, v = np.linalg.svd(matrix)
        return u @ np.diag([1, 1, np.linalg.det(u) * np.linalg.det(v)]) @ v.T

    def in_goal_region(self, x: tuple[float, ...]) -> bool:
        """Check whether a state lies in the goal region."""
        return get_dist(x, self.xgoal) <= self.rgoal

    def cost(self, x: tuple[float, ...]) -> float:
        """Compute recursive cost-to-come for a state.

        Args:
            x: Query state.

        Returns:
            Cost from start to `x`, or `np.inf` if not connected.
        """
        if x == self.xstart:
            return 0.0
        if x not in self.parent_by_node:
            return np.inf
        return self.cost(self.parent_by_node[x]) + get_dist(x, self.parent_by_node[x])

    def line(self, x: tuple[float, ...], y: tuple[float, ...]) -> float:
        """Return edge metric used by the planner."""
        return get_dist(x, y)


if __name__ == "__main__":
    planner = InformedRrtStar(show_ellipse=False)
    planner.informed_rrt()
