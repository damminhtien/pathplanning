"""3D plotting helpers for search-planner state visualization.

This module is the canonical home for search-side 3D plotting utilities.
Planner modules must remain headless and call into this module only from
external demos or visualization-specific code paths.
"""

from __future__ import annotations

from collections.abc import Sequence
from typing import Any

import numpy as np

from pathplanning.core.types import Mat, Vec
from pathplanning.viz.rrt_3d import (
    create_sphere,
    draw_block_list,
    draw_line,
    draw_obb,
    draw_spheres,
    make_transparent,
    obb_vertices,
    render_tree_state,
    set_axes_equal,
    show,
)

Edge = tuple[Vec, Vec]


def visualize_search3d_state(
    *,
    env: Any,
    edges: Sequence[Edge],
    path_edges: Sequence[Sequence[Sequence[float]]] | Mat,
    start: Vec | Sequence[float],
    goal: Vec | Sequence[float],
    view_elev: float = 65.0,
    view_azim: float = 60.0,
    pause_s: float = 0.0001,
) -> Any:
    """Render one discrete-search 3D planner snapshot and return matplotlib axis."""
    return render_tree_state(
        env=env,
        parent_edges=edges,
        path_edges=np.asarray(path_edges, dtype=float),
        start=start,
        goal=goal,
        view_elev=view_elev,
        view_azim=view_azim,
        pause_s=pause_s,
    )


__all__ = [
    "Edge",
    "create_sphere",
    "draw_spheres",
    "draw_block_list",
    "obb_vertices",
    "draw_obb",
    "draw_line",
    "set_axes_equal",
    "make_transparent",
    "render_tree_state",
    "visualize_search3d_state",
    "show",
]
