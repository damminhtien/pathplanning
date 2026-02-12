"""Demo 3D world factory for examples and benchmarks."""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray

from pathplanning.env.continuous_3d import AABB, OBB, ContinuousSpace3D, Sphere


def _rotation_matrix_zyx_deg(z_deg: float, y_deg: float, x_deg: float) -> NDArray[np.float64]:
    z_angle, y_angle, x_angle = np.deg2rad([z_deg, y_deg, x_deg])
    rz = np.array(
        [
            [np.cos(z_angle), -np.sin(z_angle), 0.0],
            [np.sin(z_angle), np.cos(z_angle), 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=float,
    )
    ry = np.array(
        [
            [np.cos(y_angle), 0.0, np.sin(y_angle)],
            [0.0, 1.0, 0.0],
            [-np.sin(y_angle), 0.0, np.cos(y_angle)],
        ],
        dtype=float,
    )
    rx = np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, np.cos(x_angle), -np.sin(x_angle)],
            [0.0, np.sin(x_angle), np.cos(x_angle)],
        ],
        dtype=float,
    )
    return rz @ ry @ rx


def build_demo_3d_world() -> tuple[ContinuousSpace3D, NDArray[np.float64], NDArray[np.float64]]:
    """Build the legacy demo scenario on top of reusable ``ContinuousSpace3D``."""
    aabbs = [
        AABB([4.0, 12.0, 0.0], [5.0, 20.0, 5.0]),
        AABB([5.5, 12.0, 0.0], [10.0, 13.0, 5.0]),
        AABB([10.0, 12.0, 0.0], [14.0, 13.0, 5.0]),
        AABB([10.0, 9.0, 0.0], [20.0, 10.0, 5.0]),
        AABB([9.0, 6.0, 0.0], [10.0, 10.0, 5.0]),
    ]
    spheres = [
        Sphere([2.0, 6.0, 2.5], 1.0),
        Sphere([14.0, 14.0, 2.5], 2.0),
    ]
    obbs = [
        OBB([5.0, 7.0, 2.5], [0.5, 2.0, 2.5], _rotation_matrix_zyx_deg(135.0, 0.0, 0.0)),
        OBB([12.0, 4.0, 2.5], [0.5, 2.0, 2.5], _rotation_matrix_zyx_deg(45.0, 0.0, 0.0)),
    ]

    start = np.array([2.0, 2.0, 2.0], dtype=float)
    goal = np.array([6.0, 16.0, 0.0], dtype=float)
    space = ContinuousSpace3D(
        lower_bound=np.array([0.0, 0.0, 0.0], dtype=float),
        upper_bound=np.array([20.0, 20.0, 5.0], dtype=float),
        aabbs=aabbs,
        spheres=spheres,
        obbs=obbs,
        goal=goal,
        goal_tolerance=0.0,
    )
    return space, start, goal
