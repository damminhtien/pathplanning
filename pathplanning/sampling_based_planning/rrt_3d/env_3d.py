"""3D environment primitives used by sampling-based planners."""

from __future__ import annotations

from collections.abc import Sequence
from typing import TypeAlias, cast

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


def get_blocks() -> NDArray[np.float64]:
    """Return axis-aligned obstacle blocks for the default benchmark map."""
    return np.array(
        [
            [4.00e00, 1.20e01, 0.00e00, 5.00e00, 2.00e01, 5.00e00],
            [5.50e00, 1.20e01, 0.00e00, 1.00e01, 1.30e01, 5.00e00],
            [1.00e01, 1.20e01, 0.00e00, 1.40e01, 1.30e01, 5.00e00],
            [1.00e01, 9.00e00, 0.00e00, 2.00e01, 1.00e01, 5.00e00],
            [9.00e00, 6.00e00, 0.00e00, 1.00e01, 1.00e01, 5.00e00],
        ],
        dtype=float,
    )


def get_balls() -> NDArray[np.float64]:
    """Return spherical obstacles for the default benchmark map."""
    return np.array([[2.0, 6.0, 2.5, 1.0], [14.0, 14.0, 2.5, 2.0]], dtype=float)


def get_aabb_pyrr(blocks: NDArray[np.float64]) -> list[Block6]:
    """Convert block bounds to the AABB representation used by ``pyrr``."""
    aabb_list: list[Block6] = []
    for block in blocks:
        aabb_list.append(np.array([np.add(block[0:3], -0.0), np.add(block[3:6], 0.0)], dtype=float))
    return aabb_list


def get_aabb_list(blocks: NDArray[np.float64]) -> list[AxisAlignedBoundingBox]:
    """Convert block bounds to ``AxisAlignedBoundingBox`` objects."""
    aabb_list: list[AxisAlignedBoundingBox] = []
    for block in blocks:
        aabb_list.append(AxisAlignedBoundingBox(block))
    return aabb_list


def add_block(block: Sequence[float] | None = None) -> NDArray[np.float64]:
    """Return a block definition to append to the environment obstacle list."""
    if block is None:
        block = [1.51e01, 0.00e00, 2.10e00, 1.59e01, 5.00e00, 6.00e00]
    return np.asarray(block, dtype=float)


class AxisAlignedBoundingBox:
    """Axis-aligned bounding box with center/extents representation."""

    def __init__(self, aabb_values: Sequence[float] | NDArray[np.float64]) -> None:
        """Create an AABB from `[xmin, ymin, zmin, xmax, ymax, zmax]`.

        Args:
            aabb_values: Bounds encoded as six coordinates.
        """
        values = np.asarray(aabb_values, dtype=float)
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
        p: Sequence[float] | NDArray[np.float64],
        e: Sequence[float] | NDArray[np.float64],
        o: NDArray[np.float64],
    ) -> None:
        """Create an OBB from center, extents, and orientation matrix.

        Args:
            p: Box center in world coordinates.
            e: Half-extents along local box axes.
            o: 3x3 local-to-world rotation matrix.
        """
        self.center: list[float] = [float(v) for v in p]
        self.extents: list[float] = [float(v) for v in e]
        self.orientation: NDArray[np.float64] = np.asarray(o, dtype=float)
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


class Environment3D:
    """Dynamic 3D world model with AABB/OBB/sphere obstacles."""

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
        """Initialize the default 3D benchmark environment.

        Args:
            xmin: Minimum x boundary.
            ymin: Minimum y boundary.
            zmin: Minimum z boundary.
            xmax: Maximum x boundary.
            ymax: Maximum y boundary.
            zmax: Maximum z boundary.
            resolution: Inflation margin for dynamic obstacle sweeps.
            blocks: Optional custom axis-aligned blocks.
            balls: Optional custom spherical obstacles.
            obb: Optional custom oriented obstacles.
            start: Optional start state.
            goal: Optional goal state.
        """
        self.resolution = float(resolution)
        self.boundary = np.array([xmin, ymin, zmin, xmax, ymax, zmax], dtype=float)
        self.blocks = np.asarray(get_blocks() if blocks is None else blocks, dtype=float)
        self.aabb = get_aabb_list(self.blocks)
        self.aabb_pyrr = get_aabb_pyrr(self.blocks)
        self.balls = np.asarray(get_balls() if balls is None else balls, dtype=float)

        default_obb = np.array(
            [
                OrientedBoundingBox(
                    [5.0, 7.0, 2.5],
                    [0.5, 2.0, 2.5],
                    rotation_matrix(np.deg2rad(135.0), 0.0, 0.0),
                ),
                OrientedBoundingBox(
                    [12.0, 4.0, 2.5],
                    [0.5, 2.0, 2.5],
                    rotation_matrix(np.deg2rad(45.0), 0.0, 0.0),
                ),
            ],
            dtype=object,
        )
        if obb is None:
            self.obb: NDArray[np.object_] = default_obb
        else:
            self.obb = np.asarray(list(obb), dtype=object)

        self.start = np.asarray([2.0, 2.0, 2.0] if start is None else start, dtype=float)
        self.goal = np.asarray([6.0, 16.0, 0.0] if goal is None else goal, dtype=float)
        self.t = 0.0

    def new_block(self) -> None:
        """Append one additional obstacle block and refresh cached AABB data."""
        new_block = add_block()
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
        """Move one AABB obstacle and return swept current/previous bounds.

        Returns ``None`` for unsupported modes.
        """
        _ = v
        if a is None:
            a = [0.0, 0.0, 0.0]
        delta = np.asarray(a, dtype=float)

        if mode != "translation":
            return None

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

        self.aabb[block_to_move].center = [
            self.aabb[block_to_move].center[0] + float(delta[0]),
            self.aabb[block_to_move].center[1] + float(delta[1]),
            self.aabb[block_to_move].center[2] + float(delta[2]),
        ]
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
        if theta is None:
            theta = [0.0, 0.0, 0.0]
        if translation is None:
            translation = [0.0, 0.0, 0.0]

        theta_vec = np.asarray(theta, dtype=float)
        translation_vec = np.asarray(translation, dtype=float)

        current_obb = cast(OrientedBoundingBox, self.obb[obb_to_move])
        previous_ref = [current_obb]

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
                [
                    float(translation_vec[0]),
                    float(translation_vec[1]),
                    float(translation_vec[2]),
                    1.0,
                ],
            ]
        )
        return current_obb, previous_ref[0]


if __name__ == "__main__":
    _new_env = Environment3D()
