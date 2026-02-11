"""3D plotting helpers re-exported from ``pathplanning.viz.rrt_3d``."""

from __future__ import annotations

from typing import Any

import numpy as np

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
)


def visualization(init_params: Any) -> None:
    """Render planner state in 3D.

    Args:
        init_params: Planner-like object that contains the current tree/path
            state and environment references.
    """
    if init_params.ind % 100 != 0 and not init_params.done:
        return

    edges = [[child, parent] for child, parent in init_params.Parent.items()]
    render_tree_state(
        env=init_params.env,
        parent_edges=edges,
        path_edges=np.asarray(init_params.Path, dtype=float),
        start=init_params.env.start,
        goal=init_params.env.goal,
    )


__all__ = [
    "create_sphere",
    "draw_spheres",
    "draw_block_list",
    "obb_vertices",
    "draw_obb",
    "draw_line",
    "visualization",
    "set_axes_equal",
    "make_transparent",
]
