"""Batch Informed Trees (BIT*) implementation for 3D planning.

References:
    Gammell, J. D., Srinivasa, S. S., and Barfoot, T. D.
    "Batch informed trees (BIT*): Sampling-based optimal planning via the
    heuristically guided search of implicit random geometric graphs." ICRA 2015.

    Gammell, J. D., Barfoot, T. D., and Srinivasa, S. S.
    "Batch Informed Trees (BIT*): Informed asymptotically optimal anytime search."
    IJRR 39(5), 2020.
"""

import copy

import numpy as np

from .env_3d import Environment3D
from .utils_3d import get_dist, is_collide, is_in_bound, is_inside, sample_free

def create_unit_sphere(radius: float = 1.0) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Create a sampled sphere mesh in Cartesian coordinates."""
    phi = np.linspace(0, 2 * np.pi, 256).reshape(256, 1)
    theta = np.linspace(0, np.pi, 256).reshape(-1, 256)
    sphere_radius = radius

    x = sphere_radius * np.sin(theta) * np.cos(phi)
    y = sphere_radius * np.sin(theta) * np.sin(phi)
    z = sphere_radius * np.cos(theta)
    return x, y, z


def draw_ellipsoid(
    ax: object,
    rotation_matrix: np.ndarray,
    scale_matrix: np.ndarray,
    x_center: np.ndarray,
) -> None:
    """Draw an ellipsoid in the world frame."""
    xs, ys, zs = create_unit_sphere()
    pts = np.array([xs, ys, zs])
    pts_in_world_frame = rotation_matrix @ scale_matrix @ pts + x_center
    ax.plot_surface(
        pts_in_world_frame[0],
        pts_in_world_frame[1],
        pts_in_world_frame[2],
        alpha=0.05,
        color="g",
    )


class BitStar:
    """BIT* planner over the legacy ``Environment3D`` obstacle model."""

    def __init__(self, show_ellipse: bool = False, rng: np.random.Generator | None = None) -> None:
        """Initialize planner state and default tuning parameters.

        Args:
            show_ellipse: Whether to expose ellipsoid state for visualization.
        """
        self.env = Environment3D()
        self.xstart, self.xgoal = tuple(self.env.start), tuple(self.env.goal)
        self.x0, self.xt = tuple(self.env.start), tuple(self.env.goal)
        self.max_iterations = 1000
        self.eta = 7

        self.samples_per_batch = 400
        self.dimension = 3
        self.g = {self.xstart: 0.0, self.xgoal: np.inf}

        self.show_ellipse = show_ellipse

        self.done = False
        self.path_edges: list[tuple[tuple[float, ...], tuple[float, ...]]] = []

        self.rotation_matrix_world = np.zeros([3, 3])
        self.ellipse_scale = np.zeros([3, 3])
        self.xcenter = np.zeros(3)
        self.iteration_index = 0
        self.rng = rng if rng is not None else np.random.default_rng()

    def run(self) -> None:
        """Execute BIT* iterations until reaching iteration budget."""
        self.vertices = {self.xstart}
        self.edges: set[tuple[tuple[float, ...], tuple[float, ...]]] = set()
        self.parent_by_node: dict[tuple[float, ...], tuple[float, ...]] = {}
        self.samples = {self.xgoal}
        self.edge_queue: set[tuple[tuple[float, ...], tuple[float, ...]]] = set()
        self.vertex_queue: set[tuple[float, ...]] = set()
        self.connection_radius = np.inf
        self.iteration_index = 0
        num_resample = 0
        while True:
            print("round " + str(self.iteration_index))
            if len(self.edge_queue) == 0 and len(self.vertex_queue) == 0:
                self.prune(self.g_t(self.xgoal))
                self.samples = self.sample(self.samples_per_batch, self.g_t(self.xgoal))
                self.samples.add(self.xgoal)
                self.previous_vertices = {v for v in self.vertices}
                self.vertex_queue = {v for v in self.vertices}
                if self.done:
                    self.connection_radius = 2
                    num_resample += 1
                else:
                    self.connection_radius = self.radius(len(self.vertices) + len(self.samples))
            while self.best_queue_value(self.vertex_queue, mode="QV") <= self.best_queue_value(
                self.edge_queue, mode="QE"
            ):
                self.expand_vertex(self.best_in_queue(self.vertex_queue, mode="QV"))
            (vm, xm) = self.best_in_queue(self.edge_queue, mode="QE")
            self.edge_queue.remove((vm, xm))
            if self.g_t(vm) + self.c_hat(vm, xm) + self.h_hat(xm) < self.g_t(self.xgoal):
                cost = self.c(vm, xm)
                if self.g_hat(vm) + cost + self.h_hat(xm) < self.g_t(self.xgoal):
                    if self.g_t(vm) + cost < self.g_t(xm):
                        if xm in self.vertices:
                            self.edges.difference_update({(v, x) for (v, x) in self.edges if x == xm})
                        else:
                            self.samples.remove(xm)
                            self.vertices.add(xm)
                            self.vertex_queue.add(xm)
                        self.g[xm] = self.g[vm] + cost
                        self.edges.add((vm, xm))
                        self.parent_by_node[xm] = vm
                        self.edge_queue.difference_update(
                            {
                                (v, x)
                                for (v, x) in self.edge_queue
                                if x == xm and (self.g_t(v) + self.c_hat(v, xm)) >= self.g_t(xm)
                            }
                        )
            else:
                self.edge_queue = set()
                self.vertex_queue = set()
            self.iteration_index += 1

            if self.xgoal in self.parent_by_node:
                print("locating path...")
                self.done = True
                self.path_edges = self.path()

            if self.iteration_index > self.max_iterations:
                break

        print("complete")
        print("number of times resampling " + str(num_resample))

    def sample(
        self,
        sample_count: int,
        cmax: float,
        bias: float = 0.05,
        xrand: set[tuple[float, ...]] | None = None,
    ) -> set[tuple[float, ...]]:
        """Sample free states from informed ellipsoid or full configuration space."""
        if xrand is None:
            xrand = set()
        print("new sample")
        if cmax < np.inf:
            cmin = get_dist(self.xgoal, self.xstart)
            xcenter = np.array(
                [
                    (self.xgoal[0] + self.xstart[0]) / 2,
                    (self.xgoal[1] + self.xstart[1]) / 2,
                    (self.xgoal[2] + self.xstart[2]) / 2,
                ]
            )
            rotation = self.rotation_to_world_frame(self.xstart, self.xgoal)
            radius = np.zeros(3)
            radius[0] = cmax / 2
            for i in range(1, 3):
                radius[i] = np.sqrt(cmax**2 - cmin**2) / 2
            scale = np.diag(radius)
            xball = self.sample_unit_ball(sample_count)
            x = (rotation @ scale @ xball).T + np.tile(xcenter, (len(xball.T), 1))
            self.rotation_matrix_world = rotation
            self.xcenter = xcenter
            self.ellipse_scale = scale
            x2 = set(
                map(
                    tuple,
                    x[
                        np.array(
                            [
                                not is_inside(self, state)
                                and is_in_bound(self.env.boundary, state)
                                for state in x
                            ]
                        )
                    ],
                )
            )
            xrand.update(x2)
            if len(x2) < sample_count:
                return self.sample(sample_count - len(x2), cmax, bias=bias, xrand=xrand)
        else:
            for _ in range(sample_count):
                xrand.add(tuple(sample_free(self, bias=bias, rng=self.rng)))
        return xrand

    def sample_unit_ball(self, n: int) -> np.ndarray:
        """Sample ``n`` points in a unit ball using spherical coordinates."""
        r = self.rng.uniform(0.0, 1.0, size=n)
        theta = self.rng.uniform(0, np.pi, size=n)
        phi = self.rng.uniform(0, 2 * np.pi, size=n)
        x = r * np.sin(theta) * np.cos(phi)
        y = r * np.sin(theta) * np.sin(phi)
        z = r * np.cos(theta)
        return np.array([x, y, z])

    def rotation_to_world_frame(
        self, xstart: tuple[float, ...], xgoal: tuple[float, ...]
    ) -> np.ndarray:
        """Compute rotation that aligns +x axis with start-goal direction."""
        d = get_dist(xstart, xgoal)
        xstart, xgoal = np.array(xstart), np.array(xgoal)
        a1 = (xgoal - xstart) / d
        matrix = np.outer(a1, [1, 0, 0])
        U, _S, V = np.linalg.svd(matrix)
        return U @ np.diag([1, 1, np.linalg.det(U) * np.linalg.det(V)]) @ V.T

    def expand_vertex(self, v: tuple[float, ...]) -> None:
        """Expand one vertex and push valid outgoing candidates into edge queue."""
        self.vertex_queue.remove(v)
        x_near = {x for x in self.samples if get_dist(x, v) <= self.connection_radius}
        self.edge_queue.update(
            {
                (v, x)
                for x in x_near
                if self.g_hat(v) + self.c_hat(v, x) + self.h_hat(x) < self.g_t(self.xgoal)
            }
        )
        if v not in self.previous_vertices:
            v_near = {w for w in self.vertices if get_dist(w, v) <= self.connection_radius}
            self.edge_queue.update(
                {
                    (v, w)
                    for w in v_near
                    if ((v, w) not in self.edges)
                    and (self.g_hat(v) + self.c_hat(v, w) + self.h_hat(w) < self.g_t(self.xgoal))
                    and (self.g_t(v) + self.c_hat(v, w) < self.g_t(w))
                }
            )

    def prune(self, c: float) -> None:
        """Prune samples/vertices/edges that cannot improve current best cost."""
        self.samples = {x for x in self.samples if self.f_hat(x) >= c}
        self.vertices.difference_update({v for v in self.vertices if self.f_hat(v) >= c})
        self.edges.difference_update({(v, w) for (v, w) in self.edges if (self.f_hat(v) > c) or (self.f_hat(w) > c)})
        self.samples.update({v for v in self.vertices if self.g_t(v) == np.inf})
        self.vertices.difference_update({v for v in self.vertices if self.g_t(v) == np.inf})

    def radius(self, q: int) -> float:
        """Return the BIT* connection radius."""
        return (
            2
            * self.eta
            * (1 + 1 / self.dimension) ** (1 / self.dimension)
            * (self.lambda_measure(self.xf_hat(self.vertices)) / self.zeta()) ** (1 / self.dimension)
            * (np.log(q) / q) ** (1 / self.dimension)
        )

    def lambda_measure(self, inputset: set[tuple[float, ...]]) -> int:
        """Return a simple cardinality-based measure for sampled sets."""
        return len(inputset)

    def xf_hat(self, xset: set[tuple[float, ...]]) -> set[tuple[float, ...]]:
        """Return states with heuristic total cost <= current best cost."""
        cbest = self.g_t(self.xgoal)
        return {x for x in xset if self.f_hat(x) <= cbest}

    def zeta(self) -> float:
        """Return 3D unit-ball volume."""
        return 4 / 3 * np.pi

    def best_in_queue(self, _inputset: set[object], mode: str):
        """Return queue element with minimum priority under BIT* ordering."""
        if mode == "QV":
            values = {state: self.g_t(state) + self.h_hat(state) for state in self.vertex_queue}
        if mode == "QE":
            values = {
                state: self.g_t(state[0]) + self.c_hat(state[0], state[1]) + self.h_hat(state[1])
                for state in self.edge_queue
            }
        if len(values) == 0:
            print(mode + "empty")
            return None
        return min(values, key=values.get)

    def best_queue_value(self, _inputset: set[object], mode: str) -> float:
        """Return minimum queue key under BIT* ordering."""
        if mode == "QV":
            values = {self.g_t(state) + self.h_hat(state) for state in self.vertex_queue}
        if mode == "QE":
            values = {
                self.g_t(state[0]) + self.c_hat(state[0], state[1]) + self.h_hat(state[1])
                for state in self.edge_queue
            }
        if len(values) == 0:
            return np.inf
        return min(values)

    def g_hat(self, v: tuple[float, ...]) -> float:
        """Heuristic lower-bound cost from start to ``v``."""
        return get_dist(self.xstart, v)

    def h_hat(self, v: tuple[float, ...]) -> float:
        """Heuristic lower-bound cost from ``v`` to goal."""
        return get_dist(self.xgoal, v)

    def f_hat(self, v: tuple[float, ...]) -> float:
        """Heuristic lower-bound total cost through ``v``."""
        return self.g_hat(v) + self.h_hat(v)

    def c(self, v: tuple[float, ...], w: tuple[float, ...]) -> float:
        """Return true edge cost if collision-free, else ``np.inf``."""
        collide, dist = is_collide(self, v, w)
        if collide:
            return np.inf
        return dist

    def c_hat(self, v: tuple[float, ...], w: tuple[float, ...]) -> float:
        """Return admissible lower-bound edge cost."""
        return get_dist(v, w)

    def g_t(self, v: tuple[float, ...]) -> float:
        """Return cached tree cost-to-come for ``v``."""
        if v not in self.g:
            self.g[v] = np.inf
        return self.g[v]

    def path(self) -> list[tuple[tuple[float, ...], tuple[float, ...]]]:
        """Backtrack and return edge list from goal to start."""
        path_edges = []
        s = self.xgoal
        i = 0
        while s != self.xstart:
            path_edges.append((s, self.parent_by_node[s]))
            s = self.parent_by_node[s]
            if i > self.samples_per_batch:
                break
            i += 1
        return path_edges

    def visualization(self) -> None:
        """Deprecated no-op. Use plotting helpers from ``pathplanning.viz.rrt_3d``."""
        return None


if __name__ == "__main__":
    new_process = BitStar(show_ellipse=False)
    new_process.run()
