"""Reusable 3D environment primitives for geometry-based planning spaces."""

from __future__ import annotations

from collections.abc import Sequence
from typing import TypeAlias

import numpy as np
from numpy.typing import NDArray

Vector3: TypeAlias = NDArray[np.float64]
Block6: TypeAlias = NDArray[np.float64]


def rotation_matrix(z_angle: float, y_angle: float, x_angle: float) -> NDArray[np.float64]:
    """Build a 3D rotation matrix from Z-Y-X intrinsic Euler angles in radians."""
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


class AxisAlignedBoundingBox:
    """Axis-aligned bounding box represented by center/extents/axes."""

    def __init__(self, bounds: Sequence[float] | NDArray[np.float64]) -> None:
        values = np.asarray(bounds, dtype=float)
        if values.shape != (6,):
            raise ValueError("AABB bounds must be shape (6,)")

        self.center: list[float] = [
            float((values[3] + values[0]) / 2),
            float((values[4] + values[1]) / 2),
            float((values[5] + values[2]) / 2),
        ]
        self.extents: list[float] = [
            float((values[3] - values[0]) / 2),
            float((values[4] - values[1]) / 2),
            float((values[5] - values[2]) / 2),
        ]
        self.axes: list[list[float]] = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]


class OrientedBoundingBox:
    """Oriented bounding box represented by center, extents, and orientation."""

    def __init__(
        self,
        center: Sequence[float] | NDArray[np.float64],
        extents: Sequence[float] | NDArray[np.float64],
        orientation: NDArray[np.float64],
    ) -> None:
        center_arr = np.asarray(center, dtype=float)
        extents_arr = np.asarray(extents, dtype=float)
        orientation_arr = np.asarray(orientation, dtype=float)

        if center_arr.shape != (3,):
            raise ValueError("center must be shape (3,)")
        if extents_arr.shape != (3,):
            raise ValueError("extents must be shape (3,)")
        if orientation_arr.shape != (3, 3):
            raise ValueError("orientation must be shape (3, 3)")

        self.center: list[float] = [float(v) for v in center_arr]
        self.extents: list[float] = [float(v) for v in extents_arr]
        self.orientation: NDArray[np.float64] = orientation_arr
        self.transform: NDArray[np.float64] = np.vstack(
            [
                np.column_stack(
                    [
                        self.orientation.T,
                        -self.orientation.T @ np.asarray(self.center, dtype=float),
                    ]
                ),
                [0.0, 0.0, 0.0, 1.0],
            ]
        )

    @property
    def P(self) -> NDArray[np.float64]:
        return np.asarray(self.center, dtype=float)

    @property
    def E(self) -> NDArray[np.float64]:
        return np.asarray(self.extents, dtype=float)

    @property
    def O(self) -> NDArray[np.float64]:
        return self.orientation

    @property
    def T(self) -> NDArray[np.float64]:
        return self.transform


def get_aabb_pyrr(blocks: NDArray[np.float64]) -> list[Block6]:
    """Convert block bounds to the AABB representation used by plotting helpers."""
    normalized = np.asarray(blocks, dtype=float)
    if normalized.size == 0:
        return []
    if normalized.ndim != 2 or normalized.shape[1] != 6:
        raise ValueError("blocks must be shape (n, 6)")

    aabb_list: list[Block6] = []
    for block in normalized:
        aabb_list.append(np.array([np.add(block[0:3], -0.0), np.add(block[3:6], 0.0)], dtype=float))
    return aabb_list


def get_aabb_list(blocks: NDArray[np.float64]) -> list[AxisAlignedBoundingBox]:
    """Convert block bounds to :class:`AxisAlignedBoundingBox` objects."""
    normalized = np.asarray(blocks, dtype=float)
    if normalized.size == 0:
        return []
    if normalized.ndim != 2 or normalized.shape[1] != 6:
        raise ValueError("blocks must be shape (n, 6)")

    return [AxisAlignedBoundingBox(block) for block in normalized]


class Environment3D:
    """Container for 3D bounds and obstacle primitives.

    This class intentionally does not define any hardcoded demo world data.
    Examples and benchmarks should construct concrete worlds in `examples/`.
    """

    def __init__(
        self,
        xmin: float = 0.0,
        ymin: float = 0.0,
        zmin: float = 0.0,
        xmax: float = 20.0,
        ymax: float = 20.0,
        zmax: float = 5.0,
        resolution: float = 1.0,
        blocks: NDArray[np.float64] | None = None,
        balls: NDArray[np.float64] | None = None,
        obb: Sequence[OrientedBoundingBox] | None = None,
        start: Sequence[float] | NDArray[np.float64] | None = None,
        goal: Sequence[float] | NDArray[np.float64] | None = None,
    ) -> None:
        self.resolution = float(resolution)
        self.boundary = np.array([xmin, ymin, zmin, xmax, ymax, zmax], dtype=float)

        block_matrix = np.asarray(
            np.empty((0, 6), dtype=float) if blocks is None else blocks,
            dtype=float,
        )
        if block_matrix.ndim != 2 or block_matrix.shape[1] != 6:
            raise ValueError("blocks must be shape (n, 6)")
        self.blocks = block_matrix

        ball_matrix = np.asarray(
            np.empty((0, 4), dtype=float) if balls is None else balls,
            dtype=float,
        )
        if ball_matrix.ndim != 2 or ball_matrix.shape[1] != 4:
            raise ValueError("balls must be shape (n, 4)")
        self.balls = ball_matrix

        self.aabb = get_aabb_list(self.blocks)
        self.aabb_pyrr = get_aabb_pyrr(self.blocks)
        self.obb: NDArray[np.object_] = np.asarray(list(obb or []), dtype=object)

        self.start = np.asarray(
            [xmin, ymin, zmin] if start is None else start,
            dtype=float,
        )
        self.goal = np.asarray(
            [xmax, ymax, zmin] if goal is None else goal,
            dtype=float,
        )
        self.t = 0.0

    def add_block(self, block: Sequence[float] | NDArray[np.float64]) -> None:
        """Append one AABB block and refresh cached AABB helpers."""
        new_block = np.asarray(block, dtype=float)
        if new_block.shape != (6,):
            raise ValueError("block must be shape (6,)")

        self.blocks = np.vstack([self.blocks, new_block])
        self.aabb = get_aabb_list(self.blocks)
        self.aabb_pyrr = get_aabb_pyrr(self.blocks)

    def move_start(self, x: Sequence[float] | NDArray[np.float64]) -> None:
        """Update the start position."""
        self.start = np.asarray(x, dtype=float)

    def move_block(
        self,
        a: Sequence[float] | None = None,
        s: float = 0.0,
        v: Sequence[float] | None = None,
        block_to_move: int = 0,
        mode: str = "translation",
    ) -> tuple[NDArray[np.float64], NDArray[np.float64]] | None:
        """Move one AABB obstacle and return swept current/previous bounds."""
        _ = v
        if mode != "translation":
            return None
        if self.blocks.shape[0] == 0:
            raise IndexError("cannot move block: no blocks configured")

        delta = np.asarray([0.0, 0.0, 0.0] if a is None else a, dtype=float)
        if delta.shape != (3,):
            raise ValueError("a must be shape (3,)")

        original = np.array(self.blocks[block_to_move], dtype=float)
        self.blocks[block_to_move] = np.array(
            [
                original[0] + delta[0],
                original[1] + delta[1],
                original[2] + delta[2],
                original[3] + delta[0],
                original[4] + delta[1],
                original[5] + delta[2],
            ],
            dtype=float,
        )

        self.aabb = get_aabb_list(self.blocks)
        self.aabb_pyrr = get_aabb_pyrr(self.blocks)
        self.t += float(s)

        moved = self.blocks[block_to_move]
        current = np.array(
            [
                moved[0] - self.resolution,
                moved[1] - self.resolution,
                moved[2] - self.resolution,
                moved[3] + self.resolution,
                moved[4] + self.resolution,
                moved[5] + self.resolution,
            ],
            dtype=float,
        )
        previous = np.array(
            [
                original[0] - self.resolution,
                original[1] - self.resolution,
                original[2] - self.resolution,
                original[3] + self.resolution,
                original[4] + self.resolution,
                original[5] + self.resolution,
            ],
            dtype=float,
        )
        return current, previous

    def move_obb(
        self,
        obb_to_move: int = 0,
        theta: Sequence[float] | None = None,
        translation: Sequence[float] | None = None,
    ) -> tuple[OrientedBoundingBox, OrientedBoundingBox]:
        """Move/rotate one OBB and return ``(new_obb, old_obb_reference)``."""
        if self.obb.size == 0:
            raise IndexError("cannot move obb: no OBB configured")

        theta_vec = np.asarray([0.0, 0.0, 0.0] if theta is None else theta, dtype=float)
        translation_vec = np.asarray(
            [0.0, 0.0, 0.0] if translation is None else translation,
            dtype=float,
        )
        if theta_vec.shape != (3,):
            raise ValueError("theta must be shape (3,)")
        if translation_vec.shape != (3,):
            raise ValueError("translation must be shape (3,)")

        current_obb = self.obb[obb_to_move]
        if not isinstance(current_obb, OrientedBoundingBox):
            raise TypeError("obb array must contain OrientedBoundingBox instances")

        old_reference = current_obb
        current_obb.center = [
            current_obb.center[0] + float(translation_vec[0]),
            current_obb.center[1] + float(translation_vec[1]),
            current_obb.center[2] + float(translation_vec[2]),
        ]
        current_obb.orientation = rotation_matrix(
            z_angle=float(theta_vec[0]),
            y_angle=float(theta_vec[1]),
            x_angle=float(theta_vec[2]),
        )
        current_obb.transform = np.vstack(
            [
                np.column_stack(
                    [
                        current_obb.orientation.T,
                        -current_obb.orientation.T @ np.asarray(current_obb.center, dtype=float),
                    ]
                ),
                [0.0, 0.0, 0.0, 1.0],
            ]
        )
        return current_obb, old_reference


__all__ = [
    "Environment3D",
    "AxisAlignedBoundingBox",
    "OrientedBoundingBox",
    "rotation_matrix",
    "get_aabb_pyrr",
    "get_aabb_list",
]
