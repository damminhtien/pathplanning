"""Native kinodynamic search adapters."""

from __future__ import annotations

import ctypes
import math
import time
from typing import Any, cast

import numpy as np

from pathplanning.core.contracts import KinematicSpace, State
from pathplanning.core.params import HybridAStarParams
from pathplanning.core.results import KinematicPlanResult, StopReason
from pathplanning.core.trace import TraceOptions
from pathplanning.native._ffi import (
    HybridAStarOptions,
    HybridAStarResult,
    HybridAStarSpace,
    HybridStateValidCallback,
    TraceResult,
    copy_trace_result,
    load_native_library,
    load_search_trace_library,
)
from pathplanning.native.continuous_model import NativeAckermannGridModel


def _pose(value: object, name: str) -> np.ndarray:
    pose = np.asarray(value, dtype=np.float64)
    if pose.shape != (3,) or not np.all(np.isfinite(pose)):
        raise ValueError(f"{name} must be a finite (x, y, yaw) pose")
    pose = np.ascontiguousarray(pose)
    pose[2] = (pose[2] + math.pi) % (2.0 * math.pi) - math.pi
    return pose


def _finite_positive(space: KinematicSpace[State], name: str) -> float:
    value = getattr(space, name)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError(f"space.{name} must be a finite real number")
    converted = float(value)
    if not math.isfinite(converted) or converted <= 0.0:
        raise ValueError(f"space.{name} must be positive and finite")
    return converted


def run_hybrid_astar(
    space: KinematicSpace[State],
    start: object,
    goal: object,
    params: HybridAStarParams,
    *,
    trace: TraceOptions | None = None,
) -> KinematicPlanResult:
    """Run native SE(2) Hybrid A* and copy owned buffers into Python arrays."""
    total_started = time.perf_counter()
    resolved = params.validate()
    start_pose = _pose(start, "start")
    goal_pose = _pose(goal, "goal")
    wheelbase = _finite_positive(space, "wheelbase")
    footprint_length = _finite_positive(space, "footprint_length")
    footprint_width = _finite_positive(space, "footprint_width")
    maximum_steering = getattr(space, "max_steering_angle")
    if isinstance(maximum_steering, bool) or not isinstance(maximum_steering, (int, float)):
        raise TypeError("space.max_steering_angle must be a finite real number")
    maximum_steering = float(maximum_steering)
    if not math.isfinite(maximum_steering) or not 0.0 < maximum_steering < math.pi / 2.0:
        raise ValueError("space.max_steering_angle must be between 0 and pi/2")

    occupancy: np.ndarray | None = None
    resolution = 1.0
    origin = np.zeros(2, dtype=np.float64)
    model_provider = getattr(space, "to_native_model", None)
    if callable(model_provider):
        model = model_provider()
        if not isinstance(model, NativeAckermannGridModel):
            raise TypeError("to_native_model() must return NativeAckermannGridModel")
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
        state_valid = getattr(space, "is_state_valid", None)
        if not callable(state_valid):
            raise TypeError("kinematic space must implement is_state_valid(state)")
        if not resolved.allow_python_callbacks:
            raise ValueError(
                "custom kinematic-space callbacks require "
                "HybridAStarParams(allow_python_callbacks=True)"
            )

    xy_resolution = resolved.xy_resolution or resolution
    primitive_length = resolved.primitive_length or 2.0 * xy_resolution
    space_collision_step = getattr(space, "collision_step", None)
    if space_collision_step is None:
        default_collision_step = min(resolution / 3.0, primitive_length / 4.0)
    else:
        if isinstance(space_collision_step, bool) or not isinstance(
            space_collision_step, (int, float)
        ):
            raise TypeError("space.collision_step must be a finite real number")
        default_collision_step = float(space_collision_step)
        if not math.isfinite(default_collision_step) or default_collision_step <= 0.0:
            raise ValueError("space.collision_step must be positive and finite")
    collision_step = (
        resolved.collision_step if resolved.collision_step is not None else default_collision_step
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
                    raise ValueError("Hybrid A* callback received a non-SE(2) state")
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
    native_space = HybridAStarSpace(
        int(grid_width),
        int(grid_height),
        occupancy_pointer,
        resolution,
        float(origin[0]),
        float(origin[1]),
        wheelbase,
        maximum_steering,
        footprint_length,
        footprint_width,
        callback,
        None,
    )
    native_options = HybridAStarOptions(
        resolved.max_expansions,
        0.0 if resolved.time_budget_s is None else resolved.time_budget_s * 1000.0,
        xy_resolution,
        resolved.heading_bins,
        primitive_length,
        collision_step,
        resolved.goal_xy_tolerance,
        resolved.goal_yaw_tolerance,
        resolved.analytic_expansion_distance,
        resolved.analytic_expansion_interval,
        resolved.heuristic_weight,
        resolved.reverse_penalty,
        resolved.steering_penalty,
        resolved.direction_switch_penalty,
        int(resolved.allow_reverse),
    )
    native_result = HybridAStarResult()
    native_trace = TraceResult() if trace is not None else None
    library = load_search_trace_library() if native_trace is not None else load_native_library()
    if native_trace is None:
        return_code = library.pp_hybrid_astar_plan(
            ctypes.byref(native_space),
            start_pose.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
            goal_pose.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
            ctypes.byref(native_options),
            ctypes.byref(native_result),
        )
    else:
        return_code = library.pp_hybrid_astar_plan_traced(
            ctypes.byref(native_space),
            start_pose.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
            goal_pose.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
            ctypes.byref(native_options),
            trace.max_bytes,
            ctypes.byref(native_result),
            ctypes.byref(native_trace),
        )

    native_trace_result = None
    try:
        if callback_error is not None:
            raise RuntimeError("kinematic state-validity callback failed") from callback_error
        if return_code != 0:
            message = native_result.error_message
            detail = (
                "unknown native Hybrid A* error" if message is None else message.decode("utf-8")
            )
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
            raise RuntimeError("native Hybrid A* returned an unknown stop reason") from exc

        path = None
        directions: tuple[int, ...] = ()
        if native_result.success:
            if (
                native_result.path_length == 0
                or not native_result.poses
                or not native_result.directions
            ):
                raise RuntimeError("native Hybrid A* returned an empty successful path")
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
            "analytic_expansions": float(native_result.analytic_expansions),
            "first_solution_iter": float(native_result.first_solution_iter),
            "path_cost": float(native_result.path_cost),
            "turning_radius": wheelbase / math.tan(maximum_steering),
            "native_space_model": float(occupancy is not None),
            "python_callbacks": float(occupancy is None),
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
        library.pp_hybrid_astar_free_result(ctypes.byref(native_result))
        if native_trace is not None:
            library.pp_search_trace_free_result(ctypes.byref(native_trace))


__all__ = ["run_hybrid_astar"]
