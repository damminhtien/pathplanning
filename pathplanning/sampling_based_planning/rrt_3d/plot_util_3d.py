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


def visualization(initparams: Any) -> None:
    """Render planner state in 3D."""
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
