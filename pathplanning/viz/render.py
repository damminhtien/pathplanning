"""Figure-owned 2D and 3D Matplotlib rendering."""

from __future__ import annotations

from collections.abc import Mapping
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray

from pathplanning.viz.replay import ReplayState
from pathplanning.viz.scene import Box, Circle, Rectangle, Scene, Sphere


def _box_faces(box: Box) -> list[NDArray[np.float64]]:
    lower = np.asarray(box.lower, dtype=float)
    upper = np.asarray(box.upper, dtype=float)
    unit = np.array(
        [[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0], [0, 0, 1], [1, 0, 1], [1, 1, 1], [0, 1, 1]],
        dtype=float,
    )
    vertices = lower + unit * (upper - lower)
    if box.orientation is not None:
        center = (lower + upper) / 2.0
        vertices = (vertices - center) @ np.asarray(box.orientation, dtype=float).T + center
    return [
        vertices[list(index)]
        for index in (
            (0, 1, 2, 3),
            (4, 5, 6, 7),
            (0, 1, 5, 4),
            (1, 2, 6, 5),
            (2, 3, 7, 6),
            (3, 0, 4, 7),
        )
    ]


def _sphere_faces(sphere: Sphere) -> list[NDArray[np.float64]]:
    center = np.asarray(sphere.center, dtype=float)
    u = np.linspace(0.0, 2.0 * np.pi, 13)
    v = np.linspace(0.0, np.pi, 9)
    points = np.stack(
        [
            np.sin(v)[:, None] * np.cos(u)[None, :],
            np.sin(v)[:, None] * np.sin(u)[None, :],
            np.broadcast_to(np.cos(v)[:, None], (len(v), len(u))),
        ],
        axis=-1,
    )
    points = center + sphere.radius * points
    return [
        points[i : i + 2, j : j + 2].reshape(4, 3)[[0, 1, 3, 2]]
        for i in range(len(v) - 1)
        for j in range(len(u) - 1)
    ]


class SceneRenderer:
    """Own the artists for one scene and update them without clearing axes."""

    def __init__(self, scene: Scene, result: Any, *, ax: Any = None) -> None:
        self.scene = scene
        self.result = result
        if ax is None:
            from matplotlib.backends.backend_agg import FigureCanvasAgg
            from matplotlib.figure import Figure

            self.figure = Figure(figsize=(9, 7))
            FigureCanvasAgg(self.figure)
            self.ax = self.figure.add_subplot(
                111, projection="3d" if scene.dimension == 3 else None
            )
        else:
            self.ax = ax
            self.figure = ax.figure
            if (getattr(ax, "name", "") == "3d") != (scene.dimension == 3):
                raise ValueError("axes projection must match scene dimension")

        self._layers: dict[str, list[Any]] = {
            "obstacles": [],
            "samples": [],
            "visited": [],
            "frontier": [],
            "tree": [],
            "path": [],
        }
        self._point_cache: dict[int, NDArray[np.float64]] = {}
        self.path_drawn = False
        self._init_axes()
        self._draw_obstacles()
        self._init_dynamic_artists()
        self._draw_endpoints()
        self._draw_final_path()

    @property
    def layers(self) -> tuple[str, ...]:
        return tuple(self._layers)

    def _init_axes(self) -> None:
        lo, hi = self.scene.bounds
        self.ax.set_xlim(float(lo[0]), float(hi[0]))
        self.ax.set_ylim(float(lo[1]), float(hi[1]))
        self.ax.set_xlabel("x")
        self.ax.set_ylabel("y")
        self.ax.set_title(self.scene.title)
        if self.scene.dimension == 3:
            self.ax.set_zlim(float(lo[2]), float(hi[2]))
            self.ax.set_zlabel("z")
            self.ax.set_box_aspect(tuple(float(v) for v in hi - lo))
        else:
            self.ax.set_aspect("equal", adjustable="box")

    def _draw_obstacles(self) -> None:
        if self.scene.dimension == 2:
            from matplotlib.collections import PatchCollection
            from matplotlib.patches import Circle as CirclePatch
            from matplotlib.patches import Rectangle as RectanglePatch

            patches = []
            for obstacle in self.scene.obstacles:
                if isinstance(obstacle, Rectangle):
                    patches.append(
                        RectanglePatch((obstacle.x, obstacle.y), obstacle.width, obstacle.height)
                    )
                elif isinstance(obstacle, Circle):
                    patches.append(CirclePatch((obstacle.x, obstacle.y), obstacle.radius))
                else:
                    raise ValueError("2D scene has a 3D obstacle")
            if patches:
                collection = PatchCollection(
                    patches, facecolor="#667085", alpha=0.5, edgecolor="#344054", linewidth=0.5
                )
                self.ax.add_collection(collection)
                self._layers["obstacles"].append(collection)
            if self.scene.occupancy is not None:
                lo, hi = self.scene.bounds
                image = self.ax.imshow(
                    self.scene.occupancy,
                    origin="lower",
                    extent=(lo[0], hi[0], lo[1], hi[1]),
                    cmap="Greys",
                    alpha=0.45,
                    interpolation="nearest",
                    vmin=0,
                    vmax=1,
                    zorder=0,
                )
                self._layers["obstacles"].append(image)
            return

        from mpl_toolkits.mplot3d.art3d import Poly3DCollection

        faces: list[NDArray[np.float64]] = []
        for obstacle in self.scene.obstacles:
            if isinstance(obstacle, Box):
                faces.extend(_box_faces(obstacle))
            elif isinstance(obstacle, Sphere):
                faces.extend(_sphere_faces(obstacle))
            else:
                raise ValueError("3D scene has a 2D obstacle")
        if faces:
            collection = Poly3DCollection(
                faces, facecolor="#667085", edgecolor="#344054", linewidth=0.2, alpha=0.3
            )
            self.ax.add_collection3d(collection)
            self._layers["obstacles"].append(collection)

    def _new_segments(self, color: str, width: float, layer: str) -> Any:
        if self.scene.dimension == 2:
            from matplotlib.collections import LineCollection

            artist = LineCollection([], colors=color, linewidths=width)
            self.ax.add_collection(artist)
        else:
            from mpl_toolkits.mplot3d.art3d import Line3DCollection

            # Matplotlib 3.11 cannot add an initially empty 3D collection.
            artist = Line3DCollection(np.zeros((1, 2, 3)), colors=color, linewidths=width)
            self.ax.add_collection3d(artist)
            artist.set_segments([])
        self._layers[layer].append(artist)
        return artist

    def _new_points(self, color: str, size: float, layer: str) -> Any:
        if self.scene.dimension == 2:
            artist = self.ax.scatter([], [], c=color, s=size, zorder=5)
        else:
            artist = self.ax.scatter([], [], [], c=color, s=size, depthshade=False)
        self._layers[layer].append(artist)
        return artist

    def _init_dynamic_artists(self) -> None:
        self._tree = (
            self._new_segments("#12b76a", 0.8, "tree"),
            self._new_segments("#12b76a", 0.8, "tree"),
            self._new_segments("#2e90fa", 0.8, "tree"),
        )
        self._path = self._new_segments("#f04438", 2.5, "path")
        self._visited = (
            self._new_points("#12b76a", 7, "visited"),
            self._new_points("#12b76a", 7, "visited"),
            self._new_points("#2e90fa", 7, "visited"),
        )
        self._frontier = (
            self._new_points("#f79009", 13, "frontier"),
            self._new_points("#f79009", 13, "frontier"),
            self._new_points("#7a5af8", 13, "frontier"),
        )
        self._samples = self._new_points("#98a2b3", 5, "samples")

    def _draw_endpoints(self) -> None:
        for point, color, marker in (
            (self.scene.start, "#12b76a", "o"),
            (self.scene.goal, "#f04438", "*"),
        ):
            if point is None:
                continue
            if self.scene.dimension == 2:
                self.ax.scatter(
                    [point[0]],
                    [point[1]],
                    c=color,
                    s=75,
                    marker=marker,
                    edgecolors="black",
                    zorder=8,
                )
            else:
                self.ax.scatter(
                    [point[0]],
                    [point[1]],
                    [point[2]],
                    c=color,
                    s=75,
                    marker=marker,
                    edgecolors="black",
                    depthshade=False,
                )

    def _draw_final_path(self) -> None:
        path = getattr(self.result, "path", None)
        if path is None:
            path = getattr(self.result, "best_path", None)
        if path is None:
            return
        try:
            points = np.asarray(path, dtype=float)
        except (TypeError, ValueError):
            return
        if points.ndim == 2 and points.shape[1] == self.scene.dimension and len(points) > 1:
            self._path.set_segments(np.stack((points[:-1], points[1:]), axis=1))
            self.path_drawn = True

    def set_path_from_trace(
        self,
        node_ids: tuple[int, ...],
        trace: Any,
        *,
        positions: Mapping[Any, Any] | None = None,
    ) -> None:
        """Show a trace-derived final path when result labels are non-coordinate."""
        if len(node_ids) < 2:
            return
        points = np.stack([self._point_for(node, trace, positions) for node in node_ids])
        self._path.set_segments(np.stack((points[:-1], points[1:]), axis=1))
        self.path_drawn = True

    def _set_points(self, artist: Any, points: list[NDArray[np.float64]]) -> None:
        data = np.asarray(points, dtype=float).reshape(-1, self.scene.dimension)
        if self.scene.dimension == 2:
            artist.set_offsets(data)
        else:
            artist._offsets3d = (data[:, 0], data[:, 1], data[:, 2])

    def _point_for(
        self,
        node_id: int,
        trace: Any,
        positions: Mapping[Any, Any] | None,
    ) -> NDArray[np.float64]:
        point = self._point_cache.get(node_id)
        if point is not None:
            return point
        values = getattr(trace, "points", None)
        if values is not None and 0 <= node_id < len(values):
            value = values[node_id]
        else:
            labels = getattr(trace, "node_labels", None)
            label = (
                labels[node_id] if labels is not None and 0 <= node_id < len(labels) else node_id
            )
            value = label
            if positions is not None:
                try:
                    value = positions[label]
                except (KeyError, TypeError):
                    value = positions.get(node_id, label)
        try:
            point = np.asarray(value, dtype=float)
        except (TypeError, ValueError) as exc:
            raise ValueError("non-coordinate graph labels require positions") from exc
        if point.shape != (self.scene.dimension,):
            raise ValueError(
                "trace node coordinates do not match scene dimension; provide positions"
            )
        self._point_cache[node_id] = point
        return point

    def show_state(
        self,
        state: ReplayState,
        trace: Any,
        *,
        positions: Mapping[Any, Any] | None = None,
    ) -> None:
        """Update existing artists from a replay snapshot; preserve camera and zoom."""
        for side in (0, 1, 2):
            edges = [
                (self._point_for(parent, trace, positions), self._point_for(node, trace, positions))
                for (edge_side, node), parent in state.parents.items()
                if edge_side == side and parent >= 0
            ]
            self._tree[side].set_segments(edges)
            self._set_points(
                self._visited[side],
                [
                    self._point_for(node, trace, positions)
                    for edge_side, node in state.visited
                    if edge_side == side
                ],
            )
            self._set_points(
                self._frontier[side],
                [
                    self._point_for(node, trace, positions)
                    for edge_side, node in state.frontier
                    if edge_side == side
                ],
            )
        self._set_points(
            self._samples,
            [self._point_for(node, trace, positions) for node in state.samples],
        )
        self.figure.canvas.draw_idle()

    def set_layer_visible(self, name: str, visible: bool) -> None:
        if name not in self._layers:
            raise KeyError(name)
        for artist in self._layers[name]:
            artist.set_visible(visible)
        self.figure.canvas.draw_idle()

    def save(self, path: str | Path, **kwargs: Any) -> None:
        target = Path(path)
        if target.suffix.lower() not in {".png", ".svg", ".pdf"}:
            raise ValueError("visualization export supports PNG, SVG, and PDF")
        self.figure.savefig(target, **kwargs)


def render_result(scene: Scene, result: Any, *, ax: Any = None) -> SceneRenderer:
    """Render a completed plan to an independent Matplotlib figure or axes."""
    return SceneRenderer(scene, result, ax=ax)


__all__ = ["SceneRenderer", "render_result"]
