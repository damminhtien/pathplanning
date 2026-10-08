"""Sample SE(2) motion primitives and occupancy-grid state-lattice space."""

from __future__ import annotations

from collections.abc import Sequence
import math

import numpy as np
from numpy.typing import NDArray

from pathplanning.core.contracts import StateLatticePrimitive
from pathplanning.core.types import Float, Vec
from pathplanning.native.continuous_model import NativePoseGridModel


def _steps(distance: float, collision_step: float) -> int:
    if not math.isfinite(distance) or distance <= 0.0:
        raise ValueError("primitive distance must be positive and finite")
    if not math.isfinite(collision_step) or collision_step <= 0.0:
        raise ValueError("collision_step must be positive and finite")
    return max(1, math.ceil(distance / collision_step))


def ackermann_motion_primitives(
    wheelbase: Float,
    max_steering_angle: Float,
    primitive_length: Float,
    *,
    collision_step: Float,
    allow_reverse: bool = True,
) -> tuple[StateLatticePrimitive, ...]:
    """Build sampled straight and maximum-steering bicycle primitives."""
    if not math.isfinite(wheelbase) or wheelbase <= 0.0:
        raise ValueError("wheelbase must be positive and finite")
    if not math.isfinite(max_steering_angle) or not 0.0 < max_steering_angle < math.pi / 2.0:
        raise ValueError("max_steering_angle must be between 0 and pi/2")
    steps = _steps(float(primitive_length), float(collision_step))
    gears = (1, -1) if allow_reverse else (1,)
    primitives: list[StateLatticePrimitive] = []
    for direction in gears:
        for steering in (-max_steering_angle, 0.0, max_steering_angle):
            curvature = math.tan(steering) / wheelbase
            poses: list[tuple[float, float, float]] = []
            for index in range(1, steps + 1):
                distance = primitive_length * index / steps
                yaw = direction * curvature * distance
                if abs(curvature) < 1e-12:
                    x = direction * distance
                    y = 0.0
                else:
                    x = math.sin(yaw) / curvature
                    y = (1.0 - math.cos(yaw)) / curvature
                poses.append((float(x), float(y), float(yaw)))
            primitives.append(
                StateLatticePrimitive(tuple(poses), direction, float(primitive_length))
            )
    return tuple(primitives)


def differential_drive_motion_primitives(
    track_width: Float,
    primitive_length: Float,
    *,
    collision_step: Float,
    allow_reverse: bool = True,
    turn_angle: Float = math.pi / 6.0,
) -> tuple[StateLatticePrimitive, ...]:
    """Build straight, constant-curvature, and in-place differential-drive moves."""
    if not math.isfinite(track_width) or track_width <= 0.0:
        raise ValueError("track_width must be positive and finite")
    if not math.isfinite(turn_angle) or not 0.0 < turn_angle < math.pi:
        raise ValueError("turn_angle must be between 0 and pi")
    length = float(primitive_length)
    if not math.isfinite(length) or length <= 0.0:
        raise ValueError("primitive_length must be positive and finite")
    gears = (1, -1) if allow_reverse else (1,)
    primitives: list[StateLatticePrimitive] = []
    for direction in gears:
        steps = _steps(length, float(collision_step))
        straight = tuple((direction * length * i / steps, 0.0, 0.0) for i in range(1, steps + 1))
        primitives.append(StateLatticePrimitive(straight, direction, length))
        radius = direction * length / turn_angle
        for turn_sign in (-1.0, 1.0):
            angle_steps = _steps(max(length, abs(radius) * turn_angle), float(collision_step))
            curve = tuple(
                (
                    radius * math.sin(turn_sign * turn_angle * i / angle_steps),
                    radius * (1.0 - math.cos(turn_sign * turn_angle * i / angle_steps)),
                    turn_sign * turn_angle * i / angle_steps,
                )
                for i in range(1, angle_steps + 1)
            )
            primitives.append(StateLatticePrimitive(curve, direction, length))

    rotation_cost = 0.5 * track_width * turn_angle
    rotation_steps = _steps(rotation_cost, float(collision_step))
    for turn_sign in (-1.0, 1.0):
        rotation = tuple(
            (0.0, 0.0, turn_sign * turn_angle * i / rotation_steps)
            for i in range(1, rotation_steps + 1)
        )
        primitives.append(StateLatticePrimitive(rotation, 1, rotation_cost))
    return tuple(primitives)


class StateLatticeGridSpace:
    """Rectangular robot footprint and user-supplied local-frame primitives."""

    def __init__(
        self,
        occupancy: NDArray[np.bool_],
        *,
        resolution: Float,
        primitives: Sequence[StateLatticePrimitive],
        footprint_length: Float,
        footprint_width: Float,
        rotation_radius: Float,
        origin: Vec = (0.0, 0.0),
        collision_step: Float | None = None,
    ) -> None:
        matrix = np.asarray(occupancy, dtype=np.bool_)
        origin_array = np.asarray(origin, dtype=np.float64)
        geometry = np.asarray(
            [resolution, footprint_length, footprint_width, rotation_radius], dtype=np.float64
        )
        if matrix.ndim != 2 or min(matrix.shape, default=0) == 0:
            raise ValueError("occupancy must be a non-empty 2D matrix")
        if origin_array.shape != (2,) or not np.all(np.isfinite(origin_array)):
            raise ValueError("origin must be a finite 2D vector")
        if not np.all(np.isfinite(geometry)) or np.any(geometry <= 0.0):
            raise ValueError(
                "grid resolution, footprint, and rotation radius must be positive and finite"
            )
        if not primitives or any(
            not isinstance(item, StateLatticePrimitive) for item in primitives
        ):
            raise ValueError("primitives must contain StateLatticePrimitive values")
        chosen_step = float(resolution / 3.0 if collision_step is None else collision_step)
        if not math.isfinite(chosen_step) or chosen_step <= 0.0:
            raise ValueError("collision_step must be positive and finite")

        self.occupancy = np.ascontiguousarray(matrix)
        self.resolution = float(resolution)
        self.origin = origin_array
        self.footprint_length = float(footprint_length)
        self.footprint_width = float(footprint_width)
        self.rotation_radius = float(rotation_radius)
        self.motion_primitives = tuple(primitives)
        self.collision_step = chosen_step

    def to_native_model(self) -> NativePoseGridModel:
        """Return an immutable-by-convention occupancy snapshot for C++ search."""
        return NativePoseGridModel(self.occupancy, self.resolution, self.origin)

    def is_state_valid(self, state: Vec) -> bool:
        """Check the complete rectangular footprint against occupied cells."""
        pose = np.asarray(state, dtype=np.float64)
        if pose.shape != (3,) or not np.all(np.isfinite(pose)):
            return False
        cosine = math.cos(float(pose[2]))
        sine = math.sin(float(pose[2]))
        half_length = self.footprint_length / 2.0
        half_width = self.footprint_width / 2.0
        extent_x = half_length * abs(cosine) + half_width * abs(sine)
        extent_y = half_length * abs(sine) + half_width * abs(cosine)
        height, width = self.occupancy.shape
        min_x, min_y = self.origin
        max_x = min_x + width * self.resolution
        max_y = min_y + height * self.resolution
        if (
            pose[0] - extent_x < min_x
            or pose[1] - extent_y < min_y
            or pose[0] + extent_x > max_x
            or pose[1] + extent_y > max_y
        ):
            return False
        x0 = max(0, math.floor((pose[0] - extent_x - min_x) / self.resolution))
        x1 = min(width - 1, math.floor((pose[0] + extent_x - min_x) / self.resolution))
        y0 = max(0, math.floor((pose[1] - extent_y - min_y) / self.resolution))
        y1 = min(height - 1, math.floor((pose[1] + extent_y - min_y) / self.resolution))
        half_cell = self.resolution / 2.0
        projection = half_cell * (abs(cosine) + abs(sine))
        for y in range(y0, y1 + 1):
            for x in range(x0, x1 + 1):
                if not self.occupancy[y, x]:
                    continue
                dx = min_x + (x + 0.5) * self.resolution - pose[0]
                dy = min_y + (y + 0.5) * self.resolution - pose[1]
                along = abs(dx * cosine + dy * sine)
                across = abs(-dx * sine + dy * cosine)
                if along <= half_length + projection and across <= half_width + projection:
                    return False
        return True


__all__ = [
    "StateLatticeGridSpace",
    "ackermann_motion_primitives",
    "differential_drive_motion_primitives",
]
