"""Compatibility wrappers for legacy RRT 3D plotting helpers.

All matplotlib-dependent rendering logic now lives in ``pathplanning.viz.rrt_3d``.
This module preserves the historical function names used by older planner code.
"""

from __future__ import annotations

from typing import Any

import numpy as np

from pathplanning.viz.rrt_3d import (
    create_sphere,
    draw_block_list as _draw_block_list,
    draw_line as _draw_line,
    draw_obb as _draw_obb,
    draw_spheres,
    make_transparent,
    obb_vertices,
    render_tree_state,
    set_axes_equal,
)


def CreateSphere(center: Any, r: float):
    """Legacy alias for sphere mesh creation."""
    return create_sphere(center, r)


def draw_Spheres(ax: Any, balls: Any) -> None:
    """Legacy alias for drawing spheres."""
    draw_spheres(ax, np.asarray(balls, dtype=float))


def draw_block_list(ax: Any, blocks: Any, color: Any = None, alpha: float = 0.15):
    """Legacy block drawing signature (``color`` kept for compatibility)."""
    _ = color
    return _draw_block_list(ax, np.asarray(blocks, dtype=float), alpha=alpha)


def obb_verts(obb: Any) -> np.ndarray:
    """Legacy alias for OBB vertex extraction."""
    return obb_vertices(obb)


def draw_obb(ax: Any, OBB: Any, color: Any = None, alpha: float = 0.15):
    """Legacy OBB drawing signature (``color`` kept for compatibility)."""
    _ = color
    return _draw_obb(ax, np.asarray(OBB, dtype=object), alpha=alpha)


def draw_line(ax: Any, SET: Any, visibility: float = 1, color: Any = None) -> None:
    """Legacy edge drawing signature."""
    _draw_line(ax, SET, visibility=float(visibility), color=color)


def visualization(initparams: Any) -> None:
    """Legacy whole-state renderer used by older demos."""
    if initparams.ind % 100 != 0 and not initparams.done:
        return

    edges = [[child, parent] for child, parent in initparams.Parent.items()]
    render_tree_state(
        env=initparams.env,
        parent_edges=edges,
        path_edges=np.asarray(initparams.Path, dtype=float),
        start=initparams.env.start,
        goal=initparams.env.goal,
    )


__all__ = [
    "CreateSphere",
    "draw_Spheres",
    "draw_block_list",
    "obb_verts",
    "draw_obb",
    "draw_line",
    "visualization",
    "set_axes_equal",
    "make_transparent",
]
