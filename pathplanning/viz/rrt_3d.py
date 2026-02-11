"""Centralized matplotlib helpers for 3D sampling-planner visualization."""

from __future__ import annotations

from collections.abc import Sequence
from typing import Any

import numpy as np

from ._lazy import lazy_import

plt = lazy_import("matplotlib.pyplot")
plt3d = lazy_import("mpl_toolkits.mplot3d")
art3d = lazy_import("mpl_toolkits.mplot3d.art3d")


def create_sphere(center: Sequence[float], radius: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    u = np.linspace(0, 2 * np.pi, 30)
    v = np.linspace(0, np.pi, 30)
    x = np.outer(np.cos(u), np.sin(v))
    y = np.outer(np.sin(u), np.sin(v))
    z = np.outer(np.ones(np.size(u)), np.cos(v))
    x, y, z = radius * x + center[0], radius * y + center[1], radius * z + center[2]
    return x, y, z


def draw_spheres(ax: Any, balls: np.ndarray) -> None:
    for ball in balls:
        xs, ys, zs = create_sphere(ball[0:3], float(ball[-1]))
        ax.plot_wireframe(xs, ys, zs, alpha=0.15, color="b")


def draw_block_list(ax: Any, blocks: np.ndarray, alpha: float = 0.15) -> Any | None:
    vertices = np.array(
        [
            [0, 0, 0],
            [1, 0, 0],
            [1, 1, 0],
            [0, 1, 0],
            [0, 0, 1],
            [1, 0, 1],
            [1, 1, 1],
            [0, 1, 1],
        ],
        dtype=float,
    )
    faces = np.array(
        [
            [0, 1, 5, 4],
            [1, 2, 6, 5],
            [2, 3, 7, 6],
            [3, 0, 4, 7],
            [0, 1, 2, 3],
            [4, 5, 6, 7],
        ],
        dtype=np.int64,
    )
    n_blocks = blocks.shape[0]
    sizes = blocks[:, 3:6] - blocks[:, :3]
    verts_list = np.zeros((8 * n_blocks, 3), dtype=float)
    face_list = np.zeros((6 * n_blocks, 4), dtype=np.int64)
    for idx in range(n_blocks):
        verts_list[idx * 8 : (idx + 1) * 8, :] = vertices * sizes[idx] + blocks[idx, :3]
        face_list[idx * 6 : (idx + 1) * 6, :] = faces + idx * 8
    if type(ax) is art3d.Poly3DCollection:  # noqa: E721 - keep legacy behavior
        ax.set_verts(verts_list[face_list])
        return None
    return ax.add_collection3d(
        art3d.Poly3DCollection(
            verts_list[face_list],
            facecolors="black",
            alpha=alpha,
            linewidths=1,
            edgecolors="k",
        )
    )


def obb_vertices(obb: Any) -> np.ndarray:
    ori_body = np.array(
        [
            [1, 1, 1],
            [-1, 1, 1],
            [-1, -1, 1],
            [1, -1, 1],
            [1, 1, -1],
            [-1, 1, -1],
            [-1, -1, -1],
            [1, -1, -1],
        ],
        dtype=float,
    )
    ori_body = np.multiply(ori_body, obb.E)
    return (obb.O @ ori_body.T).T + obb.P


def draw_obb(ax: Any, obb_items: np.ndarray, alpha: float = 0.15) -> Any | None:
    faces = np.array(
        [
            [0, 1, 5, 4],
            [1, 2, 6, 5],
            [2, 3, 7, 6],
            [3, 0, 4, 7],
            [0, 1, 2, 3],
            [4, 5, 6, 7],
        ],
        dtype=np.int64,
    )
    n_items = obb_items.shape[0]
    verts_list = np.zeros((8 * n_items, 3), dtype=float)
    face_list = np.zeros((6 * n_items, 4), dtype=np.int64)
    for idx in range(n_items):
        verts_list[idx * 8 : (idx + 1) * 8, :] = obb_vertices(obb_items[idx])
        face_list[idx * 6 : (idx + 1) * 6, :] = faces + idx * 8
    if type(ax) is art3d.Poly3DCollection:  # noqa: E721 - keep legacy behavior
        ax.set_verts(verts_list[face_list])
        return None
    return ax.add_collection3d(
        art3d.Poly3DCollection(
            verts_list[face_list],
            facecolors="black",
            alpha=alpha,
            linewidths=1,
            edgecolors="k",
        )
    )


def draw_line(ax: Any, edges: Sequence[Sequence[Sequence[float]]], visibility: float = 1.0, color: str | None = None) -> None:
    if len(edges) == 0:
        return
    for edge in edges:
        xs = edge[0][0], edge[1][0]
        ys = edge[0][1], edge[1][1]
        zs = edge[0][2], edge[1][2]
        line = plt3d.art3d.Line3D(xs, ys, zs, alpha=visibility, color=color)
        ax.add_line(line)


def set_axes_equal(ax: Any) -> None:
    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)
    plot_radius = 0.5 * max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])


def make_transparent(ax: Any) -> None:
    ax.xaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    ax.yaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    ax.zaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    ax.xaxis._axinfo["grid"]["color"] = (1, 1, 1, 0)  # type: ignore[attr-defined]
    ax.yaxis._axinfo["grid"]["color"] = (1, 1, 1, 0)  # type: ignore[attr-defined]
    ax.zaxis._axinfo["grid"]["color"] = (1, 1, 1, 0)  # type: ignore[attr-defined]


def render_tree_state(
    *,
    env: Any,
    parent_edges: Sequence[Sequence[Sequence[float]]],
    path_edges: Sequence[Sequence[Sequence[float]]],
    start: Sequence[float],
    goal: Sequence[float],
    view_elev: float = 65.0,
    view_azim: float = 60.0,
    pause_s: float = 0.0001,
) -> Any:
    """Render one 3D planner state and return the matplotlib axis."""
    ax = plt.subplot(111, projection="3d")
    ax.view_init(elev=view_elev, azim=view_azim)
    ax.clear()

    draw_spheres(ax, np.asarray(env.balls, dtype=float))
    draw_block_list(ax, np.asarray(env.blocks, dtype=float))
    obb_items = getattr(env, "obb", None)
    if obb_items is None:
        obb_items = getattr(env, "OBB", None)
    if obb_items is not None:
        draw_obb(ax, np.asarray(obb_items, dtype=object))
    draw_block_list(ax, np.array([env.boundary], dtype=float), alpha=0.0)

    draw_line(ax, parent_edges, visibility=0.75, color="g")
    draw_line(ax, path_edges, color="r")
    ax.plot(start[0:1], start[1:2], start[2:], "go", markersize=7, markeredgecolor="k")
    ax.plot(goal[0:1], goal[1:2], goal[2:], "ro", markersize=7, markeredgecolor="k")
    set_axes_equal(ax)
    make_transparent(ax)
    ax.set_axis_off()
    plt.pause(pause_s)
    return ax


def show() -> None:
    """Display pending matplotlib figures."""
    plt.show()
