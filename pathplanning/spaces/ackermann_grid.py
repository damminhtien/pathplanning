"""Ackermann vehicle model over a 2D occupancy grid."""

from __future__ import annotations

import math

import numpy as np
from numpy.typing import NDArray

from pathplanning.core.types import RNG, Float, FloatArray, Vec
from pathplanning.native.continuous_model import NativeAckermannGridModel


def _wrap_yaw(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _integrate_bicycle(
    pose: np.ndarray,
    gear: int,
    steering: float,
    distance: float,
    wheelbase: float,
) -> np.ndarray:
    curvature = math.tan(steering) / wheelbase
    yaw_delta = gear * curvature * distance
    if abs(curvature) < 1e-12:
        x = pose[0] + gear * distance * math.cos(pose[2])
        y = pose[1] + gear * distance * math.sin(pose[2])
    else:
        next_yaw = pose[2] + yaw_delta
        x = pose[0] + (math.sin(next_yaw) - math.sin(pose[2])) / curvature
        y = pose[1] + (math.cos(pose[2]) - math.cos(next_yaw)) / curvature
    return np.asarray([x, y, _wrap_yaw(float(pose[2]) + yaw_delta)], dtype=np.float64)


class AckermannGridSpace:
    """SE(2) Ackermann model with a rectangular footprint on an occupancy grid.

    States are ``(x, y, yaw)`` in world units. Occupied cells and the map
    boundary are treated as closed obstacles when checking the vehicle body.
    """

    def __init__(
        self,
        occupancy: NDArray[np.bool_],
        *,
        resolution: Float,
        origin: Vec = (0.0, 0.0),
        wheelbase: Float = 1.0,
        max_steering_angle: Float = math.radians(35.0),
        footprint_length: Float = 1.0,
        footprint_width: Float = 0.6,
        collision_step: Float | None = None,
        max_sample_tries: int = 1_000,
    ) -> None:
        matrix = np.asarray(occupancy, dtype=np.bool_)
        origin_array = np.asarray(origin, dtype=np.float64)
        geometry = np.asarray(
            [resolution, wheelbase, max_steering_angle, footprint_length, footprint_width],
            dtype=np.float64,
        )
        if matrix.ndim != 2 or min(matrix.shape, default=0) == 0:
            raise ValueError("occupancy must be a non-empty 2D matrix")
        if origin_array.shape != (2,) or not np.all(np.isfinite(origin_array)):
            raise ValueError("origin must be a finite 2D vector")
        if not np.all(np.isfinite(geometry)):
            raise ValueError("vehicle geometry and resolution must be finite")
        if resolution <= 0 or wheelbase <= 0 or footprint_length <= 0 or footprint_width <= 0:
            raise ValueError("resolution and vehicle dimensions must be > 0")
        if not 0.0 < max_steering_angle < math.pi / 2.0:
            raise ValueError("max_steering_angle must be between 0 and pi/2")
        chosen_step = float(resolution / 3.0 if collision_step is None else collision_step)
        if not math.isfinite(chosen_step) or chosen_step <= 0.0:
            raise ValueError("collision_step must be positive and finite")
        if isinstance(max_sample_tries, bool) or type(max_sample_tries) is not int:
            raise TypeError("max_sample_tries must be an integer")
        if max_sample_tries <= 0:
            raise ValueError("max_sample_tries must be > 0")

        self.occupancy = np.ascontiguousarray(matrix)
        self.resolution = float(resolution)
        self.origin = origin_array
        self.wheelbase = float(wheelbase)
        self.max_steering_angle = float(max_steering_angle)
        self.footprint_length = float(footprint_length)
        self.footprint_width = float(footprint_width)
        self.collision_step = chosen_step
        self.max_sample_tries = max_sample_tries

    @property
    def dimension(self) -> int:
        return 3

    @property
    def distance_metric(self) -> str:
        return "se2_turning_radius"

    @property
    def world_bounds(self) -> tuple[float, float, float, float]:
        height, width = self.occupancy.shape
        return (
            float(self.origin[0]),
            float(self.origin[1]),
            float(self.origin[0] + width * self.resolution),
            float(self.origin[1] + height * self.resolution),
        )

    def to_native_model(self) -> NativeAckermannGridModel:
        """Return a stable C-compatible snapshot of map occupancy."""
        return NativeAckermannGridModel(
            occupancy=self.occupancy,
            resolution=self.resolution,
            origin=self.origin,
        )

    def distance(self, first: Vec, second: Vec) -> float:
        start = np.asarray(first, dtype=np.float64)
        end = np.asarray(second, dtype=np.float64)
        if (
            start.shape != (3,)
            or end.shape != (3,)
            or not np.all(np.isfinite(start))
            or not np.all(np.isfinite(end))
        ):
            raise ValueError("Ackermann poses must be finite and contain x, y, and yaw")
        position_distance = float(np.hypot(*(end[:2] - start[:2])))
        heading_distance = (
            self.wheelbase
            / math.tan(self.max_steering_angle)
            * abs(_wrap_yaw(float(end[2] - start[2])))
        )
        return max(position_distance, heading_distance)

    def steer(self, first: Vec, target: Vec, step_size: Float) -> FloatArray:
        start = np.asarray(first, dtype=np.float64)
        goal = np.asarray(target, dtype=np.float64)
        if (
            start.shape != (3,)
            or goal.shape != (3,)
            or not np.all(np.isfinite(start))
            or not np.all(np.isfinite(goal))
        ):
            raise ValueError("Ackermann poses must be finite and contain x, y, and yaw")
        if not math.isfinite(float(step_size)) or step_size <= 0.0:
            raise ValueError("step_size must be positive and finite")
        distance = self.distance(start, goal)
        if distance <= 1e-12:
            return start.copy()
        length = min(float(step_size), distance)
        candidates = (
            _integrate_bicycle(start, gear, steering, length, self.wheelbase)
            for gear in (1, -1)
            for steering in (0.0, -self.max_steering_angle, self.max_steering_angle)
        )
        return min(candidates, key=lambda pose: self.distance(pose, goal))

    def sample_free(self, rng: RNG) -> FloatArray:
        min_x, min_y, max_x, max_y = self.world_bounds
        for _ in range(self.max_sample_tries):
            pose = np.asarray(
                [
                    rng.uniform(min_x, max_x),
                    rng.uniform(min_y, max_y),
                    rng.uniform(-math.pi, math.pi),
                ],
                dtype=np.float64,
            )
            if self.is_state_valid(pose):
                return pose
        raise RuntimeError("failed to sample a collision-free Ackermann pose")

    def is_state_valid(self, state: Vec) -> bool:
        pose = np.asarray(state, dtype=np.float64)
        if pose.shape != (3,) or not np.all(np.isfinite(pose)):
            return False
        yaw = float(pose[2])
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        half_length = 0.5 * self.footprint_length
        half_width = 0.5 * self.footprint_width
        extent_x = half_length * abs(cosine) + half_width * abs(sine)
        extent_y = half_length * abs(sine) + half_width * abs(cosine)
        min_x, min_y, max_x, max_y = self.world_bounds
        if (
            pose[0] - extent_x < min_x
            or pose[1] - extent_y < min_y
            or pose[0] + extent_x > max_x
            or pose[1] + extent_y > max_y
        ):
            return False

        height, width = self.occupancy.shape
        cell_x_min = max(0, int(math.floor((pose[0] - extent_x - min_x) / self.resolution)))
        cell_y_min = max(0, int(math.floor((pose[1] - extent_y - min_y) / self.resolution)))
        cell_x_max = min(width - 1, int(math.floor((pose[0] + extent_x - min_x) / self.resolution)))
        cell_y_max = min(
            height - 1, int(math.floor((pose[1] + extent_y - min_y) / self.resolution))
        )
        cell_half = 0.5 * self.resolution
        projection = cell_half * (abs(cosine) + abs(sine))
        for cell_y in range(cell_y_min, cell_y_max + 1):
            center_y = min_y + (cell_y + 0.5) * self.resolution
            for cell_x in range(cell_x_min, cell_x_max + 1):
                if not self.occupancy[cell_y, cell_x]:
                    continue
                center_x = min_x + (cell_x + 0.5) * self.resolution
                dx = center_x - pose[0]
                dy = center_y - pose[1]
                if abs(dx) > extent_x + cell_half or abs(dy) > extent_y + cell_half:
                    continue
                if (
                    abs(dx * cosine + dy * sine) <= half_length + projection
                    and abs(-dx * sine + dy * cosine) <= half_width + projection
                ):
                    return False
        return True

    def is_motion_valid(self, start: Vec, end: Vec) -> bool:
        first = np.asarray(start, dtype=np.float64)
        last = np.asarray(end, dtype=np.float64)
        if (
            first.shape != (3,)
            or last.shape != (3,)
            or not np.all(np.isfinite(first))
            or not np.all(np.isfinite(last))
            or not self.is_state_valid(first)
        ):
            return False
        displacement = last[:2] - first[:2]
        yaw_delta = _wrap_yaw(float(last[2] - first[2]))
        tolerance = 1e-6
        for gear in (1, -1):
            for steering in (0.0, -self.max_steering_angle, self.max_steering_angle):
                curvature = math.tan(steering) / self.wheelbase
                if abs(curvature) < 1e-12:
                    along = float(
                        displacement[0] * math.cos(first[2]) + displacement[1] * math.sin(first[2])
                    )
                    across = float(
                        -displacement[0] * math.sin(first[2]) + displacement[1] * math.cos(first[2])
                    )
                    length = gear * along
                    if abs(across) > tolerance or abs(yaw_delta) > tolerance:
                        continue
                else:
                    length = yaw_delta / (gear * curvature)
                if length < -tolerance:
                    continue
                length = max(0.0, length)
                projected = _integrate_bicycle(first, gear, steering, length, self.wheelbase)
                if (
                    np.linalg.norm(projected[:2] - last[:2]) > tolerance
                    or abs(_wrap_yaw(float(projected[2] - last[2]))) > tolerance
                ):
                    continue
                steps = max(1, math.ceil(length / self.collision_step))
                if all(
                    self.is_state_valid(
                        _integrate_bicycle(
                            first,
                            gear,
                            steering,
                            length * step / steps,
                            self.wheelbase,
                        )
                    )
                    for step in range(1, steps + 1)
                ):
                    return True
        return False


__all__ = ["AckermannGridSpace"]
