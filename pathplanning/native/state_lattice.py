"""Native adapter for state-lattice search."""

from __future__ import annotations

import ctypes
import math
import time
from typing import Any, cast

import numpy as np

from pathplanning.core.contracts import State, StateLatticePrimitive, StateLatticeSpace
from pathplanning.core.params import StateLatticeParams
from pathplanning.core.results import KinematicPlanResult, StopReason
from pathplanning.core.trace import TraceOptions
from pathplanning.native._ffi import (
    HybridAStarResult,
    HybridStateValidCallback,
    StateLatticeOptions,
    TraceResult,
    copy_trace_result,
    load_native_library,
    load_search_trace_library,
)
from pathplanning.native._ffi import (
    StateLatticePrimitive as NativePrimitive,
)
from pathplanning.native._ffi import (
    StateLatticeSpace as NativeSpace,
)
from pathplanning.native.continuous_model import NativeAckermannGridModel, NativePoseGridModel


def _pose(value: object, name: str) -> np.ndarray:
    pose = np.asarray(value, dtype=np.float64)
    if pose.shape != (3,) or not np.all(np.isfinite(pose)):
        raise ValueError(f"{name} must be a finite (x, y, yaw) pose")
    pose = np.ascontiguousarray(pose)
    pose[2] = (pose[2] + math.pi) % (2.0 * math.pi) - math.pi
    return pose


def _positive_space_value(space: StateLatticeSpace[State], name: str) -> float:
    value = getattr(space, name)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError(f"space.{name} must be a finite real number")
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(f"space.{name} must be positive and finite")
    return result


def run_state_lattice(
    space: StateLatticeSpace[State],
    start: object,
    goal: object,
    params: StateLatticeParams,
    *,
    trace: TraceOptions | None = None,
) -> KinematicPlanResult:
    """Run native A* over pose-discretized states and sampled local primitives."""
    total_started = time.perf_counter()
    resolved = params.validate()
    start_pose = _pose(start, "start")
    goal_pose = _pose(goal, "goal")
    footprint_length = _positive_space_value(space, "footprint_length")
    footprint_width = _positive_space_value(space, "footprint_width")
    rotation_radius = _positive_space_value(space, "rotation_radius")
    primitives = tuple(space.motion_primitives)
    if not primitives or any(not isinstance(item, StateLatticePrimitive) for item in primitives):
        raise ValueError("space.motion_primitives must contain StateLatticePrimitive values")

    occupancy: np.ndarray | None = None
    resolution = 1.0
    origin = np.zeros(2, dtype=np.float64)
    model_provider = getattr(space, "to_native_model", None)
    if callable(model_provider):
        model = model_provider()
        if not isinstance(model, (NativePoseGridModel, NativeAckermannGridModel)):
            raise TypeError("to_native_model() must return a compatible pose-grid model")
        occupancy = np.asarray(model.occupancy, dtype=np.uint8)
        origin = np.asarray(model.origin, dtype=np.float64)
        resolution = float(model.resolution)
        if occupancy.ndim != 2 or min(occupancy.shape, default=0) == 0:
            raise ValueError("native occupancy must be a non-empty 2D array")
        if origin.shape != (2,) or not np.all(np.isfinite(origin)):
            raise ValueError("native grid origin must be a finite 2D vector")
        if not math.isfinite(resolution) or resolution <= 0.0:
            raise ValueError("native grid resolution must be positive and finite")
        occupancy = np.ascontiguousarray(occupancy)
    else:
        if not callable(getattr(space, "is_state_valid", None)):
            raise TypeError("state-lattice space must implement is_state_valid(state)")
        if not resolved.allow_python_callbacks:
            raise ValueError(
                "custom state-validity callbacks require "
                "StateLatticeParams(allow_python_callbacks=True)"
            )

    collision_step_value = resolved.collision_step
    if collision_step_value is None:
        collision_step_value = getattr(space, "collision_step", None)
    if collision_step_value is None:
        collision_step_value = min(resolution / 3.0, 0.1)
    if isinstance(collision_step_value, bool) or not isinstance(collision_step_value, (int, float)):
        raise TypeError("collision_step must be a finite real number")
    collision_step = float(collision_step_value)
    if not math.isfinite(collision_step) or collision_step <= 0.0:
        raise ValueError("collision_step must be positive and finite")
    xy_resolution = resolved.xy_resolution or resolution

    native_primitive_storage: list[ctypes.Array[ctypes.c_double]] = []
    native_primitives = (NativePrimitive * len(primitives))()
    for index, primitive in enumerate(primitives):
        points = np.asarray(primitive.relative_poses, dtype=np.float64)
        if points.ndim != 2 or points.shape[1] != 3 or not np.all(np.isfinite(points)):
            raise ValueError("motion primitive poses must be finite x, y, yaw triples")
        flat = (ctypes.c_double * points.size)(*points.reshape(-1).tolist())
        native_primitive_storage.append(flat)
        native_primitives[index] = NativePrimitive(
            ctypes.cast(flat, ctypes.POINTER(ctypes.c_double)),
            int(points.shape[0]),
            int(primitive.direction),
            float(primitive.cost),
        )

    callback_error: BaseException | None = None
    callback = HybridStateValidCallback()
    if occupancy is None:

        def state_valid(_user_data, native_state, dimension):
            nonlocal callback_error
            if callback_error is not None:
                return -1
            try:
                if int(dimension) != 3:
                    raise ValueError("state-lattice callback received a non-SE(2) state")
                state = np.ctypeslib.as_array(native_state, shape=(3,)).copy()
                return int(bool(space.is_state_valid(cast(Any, state))))
            except BaseException as exc:
                callback_error = exc
                return -1

        callback = HybridStateValidCallback(state_valid)

    grid_height, grid_width = occupancy.shape if occupancy is not None else (0, 0)
    occupancy_pointer = (
        occupancy.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8))
        if occupancy is not None
        else ctypes.POINTER(ctypes.c_uint8)()
    )
    native_space = NativeSpace(
        int(grid_width),
        int(grid_height),
        occupancy_pointer,
        resolution,
        float(origin[0]),
        float(origin[1]),
        footprint_length,
        footprint_width,
        rotation_radius,
        callback,
        None,
    )
    native_options = StateLatticeOptions(
        resolved.max_expansions,
        0.0 if resolved.time_budget_s is None else resolved.time_budget_s * 1000.0,
        xy_resolution,
        resolved.heading_bins,
        collision_step,
        resolved.goal_xy_tolerance,
        resolved.goal_yaw_tolerance,
        resolved.heuristic_weight,
        resolved.reverse_penalty,
        resolved.direction_switch_penalty,
    )
    native_result = HybridAStarResult()
    native_trace = TraceResult() if trace is not None else None
    library = load_search_trace_library() if native_trace is not None else load_native_library()
    if native_trace is None:
        return_code = library.pp_state_lattice_plan(
            ctypes.byref(native_space),
            start_pose.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
            goal_pose.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
            native_primitives,
            len(primitives),
            ctypes.byref(native_options),
            ctypes.byref(native_result),
        )
    else:
        return_code = library.pp_state_lattice_plan_traced(
            ctypes.byref(native_space),
            start_pose.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
            goal_pose.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
            native_primitives,
            len(primitives),
            ctypes.byref(native_options),
            trace.max_bytes,
            ctypes.byref(native_result),
            ctypes.byref(native_trace),
        )

    native_trace_result = None
    try:
        if callback_error is not None:
            raise RuntimeError("state-lattice state-validity callback failed") from callback_error
        if return_code != 0:
            message = native_result.error_message
            detail = "unknown native state-lattice error" if message is None else message.decode()
            raise RuntimeError(detail)
        stop_reasons = {
            0: StopReason.SUCCESS,
            1: StopReason.MAX_ITERS,
            2: StopReason.NO_PROGRESS,
            4: StopReason.TIME_BUDGET,
        }
        try:
            stop_reason = stop_reasons[native_result.stop_reason]
        except KeyError as exc:
            raise RuntimeError("native state-lattice returned an unknown stop reason") from exc

        path = None
        directions: tuple[int, ...] = ()
        if native_result.success:
            if (
                native_result.path_length == 0
                or not native_result.poses
                or not native_result.directions
            ):
                raise RuntimeError("native state-lattice returned an empty successful path")
            path = (
                np.ctypeslib.as_array(
                    native_result.poses, shape=(int(native_result.path_length) * 3,)
                )
                .copy()
                .reshape(int(native_result.path_length), 3)
            )
            directions = tuple(
                int(value)
                for value in np.ctypeslib.as_array(
                    native_result.directions, shape=(int(native_result.path_length),)
                ).copy()
            )
        if native_trace is not None:
            native_trace_result = copy_trace_result(native_trace, kind="continuous")
        elapsed = time.perf_counter() - total_started
        stats = {
            "api_total_s": elapsed,
            "native_search_s": float(native_result.elapsed_s),
            "motion_checks": float(native_result.motion_checks),
            "first_solution_iter": float(native_result.first_solution_iter),
            "path_cost": float(native_result.path_cost),
            "native_space_model": float(occupancy is not None),
            "python_callbacks": float(occupancy is None),
            "primitive_count": float(len(primitives)),
        }
        return KinematicPlanResult(
            success=bool(native_result.success),
            path=path,
            best_path=path,
            stop_reason=stop_reason,
            iters=int(native_result.iters),
            nodes=int(native_result.nodes),
            stats=stats,
            trace=native_trace_result,
            directions=directions,
        )
    finally:
        library.pp_state_lattice_free_result(ctypes.byref(native_result))
        if native_trace is not None:
            library.pp_search_trace_free_result(ctypes.byref(native_trace))


__all__ = ["run_state_lattice"]
