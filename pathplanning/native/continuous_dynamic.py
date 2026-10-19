"""Stateful DynamicRRT3D adapters for the native C engine."""

from __future__ import annotations

import ctypes
import math
from typing import Any

import numpy as np

from pathplanning.core.contracts import ContinuousSpace, State
from pathplanning.core.trace import PlannerTrace, TraceOptions
from pathplanning.core.types import RNG
from pathplanning.native._continuous_ffi import (
    _ALGORITHMS,
    _array,
    _batch_motion_callback,
    _Callbacks,
    _copy_state,
    _Distance,
    _DynamicResult,
    _DynamicSessionResult,
    _Goal,
    _GoalDistance,
    _Manipulability,
    _MetricTensor,
    _MotionValid,
    _MotionValidBatch,
    _native_space_model,
    _Objective,
    _Options,
    _Point,
    _Result,
    _Sample,
    _SpaceModel,
    _StateValid,
    _Steer,
)
from pathplanning.native._ffi import (
    TraceResult,
    copy_trace_result,
    load_continuous_library,
    load_continuous_trace_library,
)


def _dynamic_callbacks(
    space: ContinuousSpace[State],
    rng: RNG,
    dimension: int,
    owned_space: Any,
) -> tuple[_Callbacks, tuple[Any, ...], list[BaseException]]:
    callback_errors: list[BaseException] = []

    def guarded(function, failure):
        if callback_errors:
            return failure
        try:
            return function()
        except BaseException as exc:  # ctypes callbacks cannot propagate Python exceptions.
            callback_errors.append(exc)
            return failure

    def sample(_user_data, out_state, dim):
        def call():
            value = _copy_state(space.sample_free(rng), "sample", int(dim))
            np.copyto(_array(out_state, int(dim)), value)
            return 0

        return guarded(call, -1)

    def state_valid(_user_data, point, dim):
        return guarded(lambda: int(bool(space.is_state_valid(_array(point, int(dim)).copy()))), -1)

    def motion_valid(_user_data, first, second, dim, step):
        def call():
            a, b = _array(first, int(dim)).copy(), _array(second, int(dim)).copy()
            checker = getattr(space, "is_motion_valid_with_step", None)
            valid = checker(a, b, float(step)) if callable(checker) else space.is_motion_valid(a, b)
            return int(bool(valid))

        return guarded(call, -1)

    def distance(_user_data, first, second, dim):
        return guarded(
            lambda: float(
                space.distance(_array(first, int(dim)).copy(), _array(second, int(dim)).copy())
            ),
            math.nan,
        )

    def steer(_user_data, first, target, dim, step_size, out_state):
        def call():
            value = _copy_state(
                space.steer(
                    _array(first, int(dim)).copy(),
                    _array(target, int(dim)).copy(),
                    float(step_size),
                ),
                "steered state",
                int(dim),
            )
            np.copyto(_array(out_state, int(dim)), value)
            return 0

        return guarded(call, -1)

    callback_objects = (
        _Sample(sample) if owned_space is None else _Sample(),
        _StateValid(state_valid) if owned_space is None else _StateValid(),
        _MotionValid(motion_valid) if owned_space is None else _MotionValid(),
        _batch_motion_callback(space, callback_errors)
        if owned_space is None
        else _MotionValidBatch(),
        _MetricTensor(),
        _Manipulability(),
        _Distance(distance) if owned_space is None else _Distance(),
        _Steer(steer) if owned_space is None else _Steer(),
        _Goal(),
        _GoalDistance(),
        _Objective(),
    )
    native_model_pointer = (
        ctypes.pointer(owned_space.model)
        if owned_space is not None
        else ctypes.POINTER(_SpaceModel)()
    )
    callbacks = _Callbacks(None, *callback_objects, native_model_pointer, 1, 0.0)
    return callbacks, callback_objects, callback_errors


def _dynamic_options(
    space: ContinuousSpace[State],
    config: Any,
    rng: RNG,
    max_iterations: int | None,
    prune_only: bool,
    *,
    generate_seed: bool = True,
) -> _Options:
    iteration_limit = int(config.max_iterations if max_iterations is None else max_iterations)
    if iteration_limit < 0 or iteration_limit >= np.iinfo(np.uint64).max:
        raise ValueError("dynamic RRT max_iterations is outside the supported range")
    return _Options(
        _ALGORITHMS["rrt"],
        iteration_limit,
        1,
        1,
        int(getattr(space, "max_sample_tries", 1_000)),
        0
        if prune_only or not generate_seed
        else int(rng.integers(0, np.iinfo(np.uint64).max, dtype=np.uint64)),
        float(config.step_size),
        float(config.goal_sample_probability),
        float(getattr(space, "collision_step", 0.1)),
        0.0,
        2.0,
        6.0,
        10.0,
        5.0,
        0.0,
        1,
        1.0,
        1.0,
        1,
        10,
        15,
        0.1,
        10.0,
        8,
        4,
        0.25,
        0.35,
        0.0,
        0.1,
        1e-6,
    )


def run_native_dynamic_rrt(
    space: ContinuousSpace[State],
    start: object,
    goal: object,
    initial_points: object,
    initial_parents: object,
    config: Any,
    rng: RNG,
    *,
    max_iterations: int | None = None,
    prune_only: bool = False,
    trace: TraceOptions | None = None,
    trace_sink: list[PlannerTrace] | None = None,
) -> tuple[np.ndarray, np.ndarray, bool, int, np.ndarray]:
    """Prune and grow a dynamic RRT tree in C, returning its compact state."""
    if trace is not None and not isinstance(trace, TraceOptions):
        raise TypeError("trace must be TraceOptions or None")
    start_array = np.asarray(start, dtype=np.float64)
    if start_array.ndim != 1 or start_array.size != 3:
        raise ValueError("DynamicRRT3D requires three-dimensional start states")
    dimension = int(start_array.size)
    start_state = _copy_state(start_array, "start", dimension)
    goal_state = _copy_state(goal, "goal", dimension)
    points = np.ascontiguousarray(initial_points, dtype=np.float64).reshape(-1, dimension)
    parent_values = np.asarray(initial_parents, dtype=np.int64)
    if points.shape[0] == 0 or parent_values.shape != (points.shape[0],):
        raise ValueError("initial dynamic RRT tree and parents must have matching non-empty sizes")
    if np.any(parent_values < -1) or np.any(parent_values >= np.arange(points.shape[0])):
        raise ValueError("dynamic RRT parents must refer to an earlier node")
    parents = np.where(parent_values < 0, np.iinfo(np.uint32).max, parent_values).astype(
        np.uint32, copy=False
    )
    owned_space = _native_space_model(space, dimension)
    allow_python_callbacks = getattr(config, "allow_python_callbacks", False)
    if type(allow_python_callbacks) is not bool:
        raise TypeError("allow_python_callbacks must be a bool")
    if owned_space is None and not allow_python_callbacks:
        raise ValueError(
            "DynamicRRT3D requires a built-in or declared native space model. "
            "Implement to_native_model() or set allow_python_callbacks=True."
        )
    callbacks, _callback_objects, callback_errors = _dynamic_callbacks(
        space, rng, dimension, owned_space
    )
    options = _dynamic_options(space, config, rng, max_iterations, prune_only)
    library = load_continuous_library() if trace is None else load_continuous_trace_library()
    library.pp_dynamic_rrt_plan.argtypes = [
        ctypes.POINTER(_Callbacks),
        _Point,
        _Point,
        _Point,
        ctypes.POINTER(ctypes.c_uint32),
        ctypes.c_size_t,
        ctypes.c_size_t,
        ctypes.POINTER(_Options),
        ctypes.c_double,
        ctypes.c_int,
        ctypes.POINTER(_DynamicResult),
    ]
    library.pp_dynamic_rrt_plan.restype = ctypes.c_int
    library.pp_dynamic_rrt_free_result.argtypes = [ctypes.POINTER(_DynamicResult)]
    library.pp_dynamic_rrt_free_result.restype = None
    if trace is not None:
        library.pp_dynamic_rrt_plan_traced.argtypes = [
            *library.pp_dynamic_rrt_plan.argtypes,
            ctypes.c_uint64,
            ctypes.POINTER(TraceResult),
        ]
        library.pp_dynamic_rrt_plan_traced.restype = ctypes.c_int
        library.pp_continuous_trace_free_result.argtypes = [ctypes.POINTER(TraceResult)]
        library.pp_continuous_trace_free_result.restype = None
    native_result = _DynamicResult()
    trace_result = TraceResult()
    call_args = (
        ctypes.byref(callbacks),
        start_state.ctypes.data_as(_Point),
        goal_state.ctypes.data_as(_Point),
        points.ctypes.data_as(_Point),
        parents.ctypes.data_as(ctypes.POINTER(ctypes.c_uint32)),
        len(points),
        dimension,
        ctypes.byref(options),
        float(config.way_point_sample_probability),
        int(prune_only),
        ctypes.byref(native_result),
    )
    status = (
        library.pp_dynamic_rrt_plan(*call_args)
        if trace is None
        else library.pp_dynamic_rrt_plan_traced(
            *call_args, ctypes.c_uint64(trace.max_bytes), ctypes.byref(trace_result)
        )
    )
    try:
        if callback_errors:
            raise callback_errors[0]
        if status != 0 or native_result.plan.stop_reason == 4:
            message = (
                native_result.plan.error_message.decode("utf-8", errors="replace")
                if native_result.plan.error_message
                else "native dynamic RRT failed"
            )
            raise RuntimeError(message)
        count = int(native_result.tree_count)
        states = (
            np.ctypeslib.as_array(native_result.tree_points, shape=(count * dimension,))
            .copy()
            .reshape(count, dimension)
        )
        parent_ids = np.ctypeslib.as_array(native_result.tree_parents, shape=(count,)).copy()
        invalid_nodes = np.ctypeslib.as_array(
            native_result.invalid_nodes,
            shape=(int(native_result.invalid_count),),
        ).copy()
        if trace is not None and trace_sink is not None:
            trace_sink.append(copy_trace_result(trace_result, kind="continuous"))
        return (
            states,
            parent_ids,
            bool(native_result.plan.success),
            int(native_result.plan.iters),
            invalid_nodes,
        )
    finally:
        library.pp_dynamic_rrt_free_result(ctypes.byref(native_result))
        if trace is not None:
            library.pp_continuous_trace_free_result(ctypes.byref(trace_result))


def _configure_session_library(library: Any) -> None:
    library.pp_dynamic_rrt_session_create.argtypes = [
        ctypes.POINTER(_Callbacks),
        _Point,
        _Point,
        _Point,
        ctypes.POINTER(ctypes.c_uint32),
        ctypes.c_size_t,
        ctypes.c_size_t,
        ctypes.POINTER(_Options),
        ctypes.c_double,
        ctypes.POINTER(ctypes.c_void_p),
        ctypes.POINTER(_Result),
    ]
    library.pp_dynamic_rrt_session_create.restype = ctypes.c_int
    library.pp_dynamic_rrt_session_run.argtypes = [
        ctypes.c_void_p,
        ctypes.POINTER(_Callbacks),
        _Point,
        _Point,
        ctypes.POINTER(_Options),
        ctypes.c_double,
        ctypes.c_int,
        ctypes.c_int,
        ctypes.POINTER(_DynamicSessionResult),
    ]
    library.pp_dynamic_rrt_session_run.restype = ctypes.c_int
    library.pp_dynamic_rrt_session_free.argtypes = [ctypes.c_void_p]
    library.pp_dynamic_rrt_session_free.restype = None
    library.pp_dynamic_rrt_session_free_result.argtypes = [ctypes.POINTER(_DynamicSessionResult)]
    library.pp_dynamic_rrt_session_free_result.restype = None
    library.pp_continuous_free_result.argtypes = [ctypes.POINTER(_Result)]
    library.pp_continuous_free_result.restype = None


def _same_space_model(first: Any, second: Any) -> bool:
    if first is None or second is None:
        return first is second
    return len(first.arrays) == len(second.arrays) and all(
        np.array_equal(left, right) for left, right in zip(first.arrays, second.arrays, strict=True)
    )


class NativeDynamicRRTSession:
    """Own a native DynamicRRT tree and synchronize only changed nodes."""

    def __init__(
        self,
        space: ContinuousSpace[State],
        start: object,
        goal: object,
        initial_points: object,
        initial_parents: object,
        config: Any,
        rng: RNG,
    ) -> None:
        self.space = space
        self.rng = rng
        self._start = _copy_state(start, "start", 3)
        self._goal = _copy_state(goal, "goal", 3)
        points = np.ascontiguousarray(initial_points, dtype=np.float64).reshape(-1, 3)
        self.node_count = int(points.shape[0])
        parent_values = np.asarray(initial_parents, dtype=np.int64)
        if points.shape[0] == 0 or parent_values.shape != (points.shape[0],):
            raise ValueError(
                "initial dynamic RRT tree and parents must have matching non-empty sizes"
            )
        if np.any(parent_values < -1) or np.any(parent_values >= np.arange(points.shape[0])):
            raise ValueError("dynamic RRT parents must refer to an earlier node")
        parents = np.where(parent_values < 0, np.iinfo(np.uint32).max, parent_values).astype(
            np.uint32, copy=False
        )
        self._owned_space = _native_space_model(space, 3)
        allow_python_callbacks = getattr(config, "allow_python_callbacks", False)
        if type(allow_python_callbacks) is not bool:
            raise TypeError("allow_python_callbacks must be a bool")
        if self._owned_space is None and not allow_python_callbacks:
            raise ValueError(
                "DynamicRRT3D requires a built-in or declared native space model. "
                "Implement to_native_model() or set allow_python_callbacks=True."
            )
        self._callbacks, self._callback_objects, self._callback_errors = _dynamic_callbacks(
            space, rng, 3, self._owned_space
        )
        self._library = load_continuous_library()
        _configure_session_library(self._library)
        self._handle = ctypes.c_void_p()
        self._library_result = _Result()
        options = _dynamic_options(space, config, rng, None, False, generate_seed=False)
        status = self._library.pp_dynamic_rrt_session_create(
            ctypes.byref(self._callbacks),
            self._start.ctypes.data_as(_Point),
            self._goal.ctypes.data_as(_Point),
            points.ctypes.data_as(_Point),
            parents.ctypes.data_as(ctypes.POINTER(ctypes.c_uint32)),
            len(points),
            3,
            ctypes.byref(options),
            float(config.way_point_sample_probability),
            ctypes.byref(self._handle),
            ctypes.byref(self._library_result),
        )
        try:
            if self._callback_errors:
                raise self._callback_errors[0]
            if status != 0 or not self._handle.value:
                message = (
                    self._library_result.error_message.decode("utf-8", errors="replace")
                    if self._library_result.error_message
                    else "native dynamic RRT session creation failed"
                )
                raise RuntimeError(message)
        except BaseException:
            self.close()
            raise
        finally:
            self._library.pp_continuous_free_result(ctypes.byref(self._library_result))

    def matches(
        self, space: ContinuousSpace[State], start: object, goal: object, node_count: int
    ) -> bool:
        return (
            self._handle.value is not None
            and space is self.space
            and node_count == self.node_count
            and np.array_equal(np.asarray(start, dtype=np.float64), self._start)
            and np.array_equal(np.asarray(goal, dtype=np.float64), self._goal)
        )

    @property
    def uses_python_callbacks(self) -> bool:
        return self._owned_space is None

    def run(
        self,
        config: Any,
        *,
        max_iterations: int | None = None,
        prune_tree: bool = False,
        prune_only: bool = False,
    ) -> tuple[np.ndarray, np.ndarray, bool, int, np.ndarray, int, bool]:
        if self._handle.value is None:
            raise RuntimeError("native dynamic RRT session is closed")
        candidate_space = _native_space_model(self.space, 3)
        changed = not _same_space_model(self._owned_space, candidate_space)
        owned_space = candidate_space if changed else self._owned_space
        callbacks, callback_objects, callback_errors = _dynamic_callbacks(
            self.space, self.rng, 3, owned_space
        )
        options = _dynamic_options(self.space, config, self.rng, max_iterations, prune_only)
        native_result = _DynamicSessionResult()
        should_prune = prune_tree or changed
        status = self._library.pp_dynamic_rrt_session_run(
            self._handle,
            ctypes.byref(callbacks),
            self._start.ctypes.data_as(_Point),
            self._goal.ctypes.data_as(_Point),
            ctypes.byref(options),
            float(config.way_point_sample_probability),
            int(should_prune),
            int(prune_only),
            ctypes.byref(native_result),
        )
        try:
            if callback_errors:
                raise callback_errors[0]
            if status != 0 or native_result.plan.stop_reason == 4:
                message = (
                    native_result.plan.error_message.decode("utf-8", errors="replace")
                    if native_result.plan.error_message
                    else "native dynamic RRT session failed"
                )
                raise RuntimeError(message)
            count = int(native_result.tree_count)
            states = (
                np.ctypeslib.as_array(native_result.tree_points, shape=(count * 3,))
                .copy()
                .reshape(count, 3)
                if count
                else np.empty((0, 3), dtype=np.float64)
            )
            parents = (
                np.ctypeslib.as_array(native_result.tree_parents, shape=(count,)).copy()
                if count
                else np.empty((0,), dtype=np.uint32)
            )
            invalid_count = int(native_result.invalid_count)
            invalid_nodes = (
                np.ctypeslib.as_array(native_result.invalid_nodes, shape=(invalid_count,)).copy()
                if invalid_count
                else np.empty((0,), dtype=np.uint32)
            )
            self._owned_space = owned_space
            self._callbacks = callbacks
            self._callback_objects = callback_objects
            self.node_count = int(native_result.plan.nodes)
            return (
                states,
                parents,
                bool(native_result.plan.success),
                int(native_result.plan.iters),
                invalid_nodes,
                int(native_result.tree_offset),
                bool(native_result.tree_is_snapshot),
            )
        except BaseException:
            self.close()
            raise
        finally:
            self._library.pp_dynamic_rrt_session_free_result(ctypes.byref(native_result))

    def close(self) -> None:
        if getattr(self, "_handle", None) is not None and self._handle.value:
            self._library.pp_dynamic_rrt_session_free(self._handle)
            self._handle = ctypes.c_void_p()

    def __del__(self) -> None:
        self.close()


__all__ = ["NativeDynamicRRTSession", "run_native_dynamic_rrt"]
