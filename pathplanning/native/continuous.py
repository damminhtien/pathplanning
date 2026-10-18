"""Typed Python adapters for native continuous planning and roadmaps."""

from __future__ import annotations

import ctypes
import math
import time
from typing import Any

import numpy as np

from pathplanning.core.contracts import ContinuousSpace, GoalRegion, Objective, State
from pathplanning.core.params import JitParams, RitParams, RoadmapParams, RrtParams
from pathplanning.core.results import PlanResult, StopReason
from pathplanning.core.trace import TraceOptions
from pathplanning.core.types import RNG
from pathplanning.native._continuous_ffi import (
    _ALGORITHMS,
    _STOP_REASONS,
    _array,
    _batch_motion_callback,
    _Callbacks,
    _copy_state,
    _Distance,
    _Goal,
    _GoalDistance,
    _Manipulability,
    _MetricTensor,
    _MotionValid,
    _MotionValidBatch,
    _native_space_model,
    _Objective,
    _Options,
    _OwnedSpaceModel,
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


def _roadmap_callbacks(
    space: ContinuousSpace[State], rng: RNG, dimension: int, allow_python_callbacks: bool
) -> tuple[_Callbacks, tuple[Any, ...], _OwnedSpaceModel | None, list[BaseException]]:
    """Build borrowed callbacks for one PRM* native call."""
    owned_space = _native_space_model(space, dimension)
    if owned_space is None and not allow_python_callbacks:
        raise ValueError(
            "Python callbacks are disabled for native planning (space has no native model). "
            "Implement to_native_model() or set allow_python_callbacks=True."
        )
    callback_errors: list[BaseException] = []

    def guarded(function, failure):
        if callback_errors:
            return failure
        try:
            return function()
        except BaseException as exc:
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
            a = _array(first, int(dim)).copy()
            b = _array(second, int(dim)).copy()
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
        _Steer(),
        _Goal(),
        _GoalDistance(),
        _Objective(),
    )
    model_pointer = (
        ctypes.pointer(owned_space.model) if owned_space else ctypes.POINTER(_SpaceModel)()
    )
    callbacks = _Callbacks(None, *callback_objects, model_pointer, 0, 0.0)
    return callbacks, callback_objects, owned_space, callback_errors


class NativePrmStarRoadmap:
    """Owner for a reusable C PRM* roadmap and its optional trace mirror."""

    def __init__(
        self,
        space: ContinuousSpace[State],
        dimension: int,
        params: RoadmapParams,
        rng: RNG,
        *,
        lazy: bool = False,
        effort_informed: bool = False,
    ) -> None:
        self.space = space
        self.dimension = dimension
        self.params = params.validate()
        self.lazy = lazy
        self.effort_informed = effort_informed
        self.seed = int(rng.integers(0, np.iinfo(np.uint64).max, dtype=np.uint64))
        self.library = load_continuous_library()
        self.handle = self._create_handle(self.library)
        self.trace_library = None
        self.trace_handle = ctypes.c_void_p()
        self.trace_built = False
        self.world_version: object = object()
        self.built = False
        self.closed = False
        self.build_stats: dict[str, float] = {}

    def _configure_library(self, library, *, traced: bool) -> None:
        library.pp_prm_star_create.argtypes = [
            ctypes.c_size_t,
            ctypes.c_uint64,
            ctypes.c_double,
            ctypes.c_uint64,
            ctypes.c_uint64,
            ctypes.POINTER(ctypes.c_void_p),
            ctypes.POINTER(ctypes.c_char),
            ctypes.c_size_t,
        ]
        library.pp_prm_star_create.restype = ctypes.c_int
        library.pp_prm_star_set_lazy.argtypes = [ctypes.c_void_p, ctypes.c_int]
        library.pp_prm_star_set_lazy.restype = ctypes.c_int
        library.pp_prm_star_set_eirm.argtypes = [ctypes.c_void_p, ctypes.c_int]
        library.pp_prm_star_set_eirm.restype = ctypes.c_int
        library.pp_prm_star_build.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(_Callbacks),
            ctypes.c_double,
            ctypes.c_double,
            ctypes.POINTER(_Result),
        ]
        library.pp_prm_star_build.restype = ctypes.c_int
        library.pp_prm_star_query.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(_Callbacks),
            _Point,
            _Point,
            ctypes.c_double,
            ctypes.c_uint64,
            ctypes.c_double,
            ctypes.POINTER(_Result),
        ]
        library.pp_prm_star_query.restype = ctypes.c_int
        library.pp_prm_star_clear_query.argtypes = [ctypes.c_void_p]
        library.pp_prm_star_clear_query.restype = None
        library.pp_prm_star_reset.argtypes = [ctypes.c_void_p]
        library.pp_prm_star_reset.restype = None
        library.pp_prm_star_free.argtypes = [ctypes.c_void_p]
        library.pp_prm_star_free.restype = None
        library.pp_continuous_free_result.argtypes = [ctypes.POINTER(_Result)]
        library.pp_continuous_free_result.restype = None
        if traced:
            library.pp_prm_star_query_traced.argtypes = [
                *library.pp_prm_star_query.argtypes,
                ctypes.c_uint64,
                ctypes.POINTER(TraceResult),
            ]
            library.pp_prm_star_query_traced.restype = ctypes.c_int
            library.pp_continuous_trace_free_result.argtypes = [ctypes.POINTER(TraceResult)]
            library.pp_continuous_trace_free_result.restype = None

    def _create_handle(self, library) -> ctypes.c_void_p:
        self._configure_library(library, traced=library is not self.library)
        handle = ctypes.c_void_p()
        error = ctypes.create_string_buffer(256)
        status = library.pp_prm_star_create(
            self.dimension,
            self.params.sample_count,
            self.params.gamma,
            self.params.max_sample_tries,
            self.seed,
            ctypes.byref(handle),
            error,
            len(error),
        )
        if status != 0 or not handle.value:
            message = (
                error.value.decode("utf-8", errors="replace") or "could not create PRM* roadmap"
            )
            raise RuntimeError(message)
        if library.pp_prm_star_set_lazy(handle, int(self.lazy)) != 0:
            library.pp_prm_star_free(handle)
            raise RuntimeError("could not configure native PRM* validation mode")
        if library.pp_prm_star_set_eirm(handle, int(self.effort_informed)) != 0:
            library.pp_prm_star_free(handle)
            raise RuntimeError("could not configure native EIRM* mode")
        return handle

    def _build_handle(self, library, handle: ctypes.c_void_p) -> dict[str, float]:
        callbacks, callback_objects, owned_space, callback_errors = _roadmap_callbacks(
            self.space,
            np.random.default_rng(self.seed),
            self.dimension,
            self.params.allow_python_callbacks,
        )
        callback_lifetime = callback_objects, owned_space
        result = _Result()
        started = time.perf_counter()
        status = library.pp_prm_star_build(
            handle,
            ctypes.byref(callbacks),
            self.params.collision_step,
            0.0 if self.params.time_budget_s is None else self.params.time_budget_s,
            ctypes.byref(result),
        )
        elapsed = time.perf_counter() - started
        try:
            if callback_errors:
                raise callback_errors[0]
            if status != 0 or result.stop_reason == 4 or not result.success:
                message = (
                    result.error_message.decode("utf-8", errors="replace")
                    if result.error_message
                    else "native PRM* roadmap build failed"
                )
                raise RuntimeError(message)
            return {
                "roadmap_vertices": float(result.nodes),
                "roadmap_motion_checks": float(result.motion_checks),
                "roadmap_connection_candidates": float(result.iters),
                "roadmap_build_s": float(result.elapsed_s),
                "roadmap_api_s": elapsed,
            }
        finally:
            library.pp_continuous_free_result(ctypes.byref(result))
            _ = callback_lifetime

    def build(self, *, world_version: object = 0) -> None:
        """Build once per caller-managed world version; reuse matching roadmaps."""
        if self.closed:
            raise RuntimeError("PRM* roadmap is closed")
        if self.built and self.world_version == world_version:
            return
        if self.built:
            self.library.pp_prm_star_reset(self.handle)
            if self.trace_handle.value:
                self.trace_library.pp_prm_star_reset(self.trace_handle)
        self.build_stats = self._build_handle(self.library, self.handle)
        self.built = True
        self.world_version = world_version
        if self.trace_handle.value:
            self.trace_built = False
            self._build_handle(self.trace_library, self.trace_handle)
            self.trace_built = True

    def _ensure_trace_handle(self) -> None:
        if not self.trace_handle.value:
            self.trace_library = load_continuous_trace_library()
            self.trace_handle = self._create_handle(self.trace_library)
        if self.built and not self.trace_built:
            self._build_handle(self.trace_library, self.trace_handle)
            self.trace_built = True

    def query(
        self,
        start: object,
        goal: object,
        *,
        world_version: object = 0,
        trace: TraceOptions | None = None,
    ) -> PlanResult:
        """Query the retained roadmap without deleting its built samples."""
        if trace is not None and type(trace) is not TraceOptions:
            raise TypeError("trace must be TraceOptions or None")
        self.build(world_version=world_version)
        start_state = _copy_state(start, "start", self.dimension)
        goal_state = _copy_state(goal, "goal", self.dimension)
        callbacks, callback_objects, owned_space, callback_errors = _roadmap_callbacks(
            self.space,
            np.random.default_rng(self.seed),
            self.dimension,
            self.params.allow_python_callbacks,
        )
        callback_lifetime = callback_objects, owned_space
        native_result = _Result()
        trace_result = TraceResult() if trace is not None else None
        library = self.library
        handle = self.handle
        if trace_result is not None:
            self._ensure_trace_handle()
            library = self.trace_library
            handle = self.trace_handle
        args = [
            handle,
            ctypes.byref(callbacks),
            start_state.ctypes.data_as(_Point),
            goal_state.ctypes.data_as(_Point),
            self.params.collision_step,
            self.params.max_expansions,
            0.0 if self.params.time_budget_s is None else self.params.time_budget_s,
            ctypes.byref(native_result),
        ]
        if trace_result is not None:
            assert trace is not None
            args.extend((trace.max_bytes, ctypes.byref(trace_result)))
        native_started = time.perf_counter()
        status = (
            library.pp_prm_star_query_traced(*args)
            if trace_result is not None
            else library.pp_prm_star_query(*args)
        )
        query_s = time.perf_counter() - native_started
        try:
            if callback_errors:
                raise callback_errors[0]
            if status != 0 or native_result.stop_reason == 4:
                message = (
                    native_result.error_message.decode("utf-8", errors="replace")
                    if native_result.error_message
                    else "native PRM* query failed"
                )
                raise RuntimeError(message)
            path = None
            if native_result.success and native_result.path:
                flat_path = np.ctypeslib.as_array(
                    native_result.path,
                    shape=(int(native_result.path_length) * self.dimension,),
                ).copy()
                path = flat_path.reshape(int(native_result.path_length), self.dimension)
            planner_trace = (
                copy_trace_result(trace_result, kind="continuous")
                if trace_result is not None
                else None
            )
            return PlanResult(
                success=bool(native_result.success),
                path=path,
                best_path=path if native_result.success else None,
                stop_reason=_STOP_REASONS.get(
                    int(native_result.stop_reason), StopReason.NO_PROGRESS
                ),
                iters=int(native_result.iters),
                nodes=int(native_result.nodes),
                stats={
                    **self.build_stats,
                    "expanded": float(native_result.iters),
                    "sample_count": float(native_result.sample_count),
                    "motion_checks": float(native_result.motion_checks),
                    "query_s": query_s,
                    "path_cost": float(native_result.path_cost) if native_result.success else 0.0,
                },
                trace=planner_trace,
            )
        finally:
            library.pp_continuous_free_result(ctypes.byref(native_result))
            if trace_result is not None:
                library.pp_continuous_trace_free_result(ctypes.byref(trace_result))
            _ = callback_lifetime

    def clear_query(self) -> None:
        """Clear query-local native state while preserving the roadmap."""
        if self.closed:
            raise RuntimeError("PRM* roadmap is closed")
        self.library.pp_prm_star_clear_query(self.handle)
        if self.trace_handle.value:
            self.trace_library.pp_prm_star_clear_query(self.trace_handle)

    def reset(self) -> None:
        """Discard samples and validation data but keep this planner object."""
        if self.closed:
            raise RuntimeError("PRM* roadmap is closed")
        self.library.pp_prm_star_reset(self.handle)
        if self.trace_handle.value:
            self.trace_library.pp_prm_star_reset(self.trace_handle)
        self.built = False
        self.trace_built = False
        self.build_stats = {}

    def close(self) -> None:
        """Release native roadmap resources."""
        if self.closed:
            return
        if self.handle.value:
            self.library.pp_prm_star_free(self.handle)
            self.handle = ctypes.c_void_p()
        if self.trace_handle.value:
            self.trace_library.pp_prm_star_free(self.trace_handle)
            self.trace_handle = ctypes.c_void_p()
        self.closed = True
        self.built = False
        self.trace_built = False

    def __del__(self) -> None:
        try:
            self.close()
        except BaseException:
            pass

    def __enter__(self) -> NativePrmStarRoadmap:
        return self

    def __exit__(self, *_exc: object) -> None:
        self.close()


def run_native_continuous(
    space: ContinuousSpace[State],
    start: object,
    goal_region: GoalRegion[State],
    params: RrtParams | RitParams | JitParams,
    rng: RNG,
    *,
    planner: str,
    objective: Objective[State] | None = None,
    trace: TraceOptions | None = None,
) -> PlanResult:
    """Run a planner in C, adapting space operations at the API boundary."""
    if trace is not None and not isinstance(trace, TraceOptions):
        raise TypeError("trace must be TraceOptions or None")
    if planner not in _ALGORITHMS:
        raise KeyError(f"Unknown native continuous planner: {planner}")
    parameters = params.validate()
    start_array = np.asarray(start, dtype=np.float64)
    if start_array.ndim != 1 or start_array.size == 0:
        raise ValueError("start must be a non-empty 1D state vector")
    dimension = int(start_array.size)
    start_state = _copy_state(start_array, "start", dimension)
    owned_space = _native_space_model(space, dimension)
    metric_function = getattr(space, "metric_tensor", None) if planner == "rit_star" else None
    manipulability_function = (
        getattr(space, "jacobian", None)
        if planner == "jit_star" and getattr(parameters, "manipulability_weight", 0.0) > 0.0
        else None
    )
    if manipulability_function is not None and not callable(manipulability_function):
        raise TypeError("space.jacobian must be callable")
    metric_bounds = (1.0, 1.0)
    metric_is_constant = True
    if planner == "rit_star":
        metric_bounds = getattr(space, "metric_eigenvalue_bounds", (1.0, 1.0))
        metric_is_constant = getattr(space, "metric_is_constant", metric_function is None)
        if (
            not isinstance(metric_bounds, (tuple, list))
            or len(metric_bounds) != 2
            or any(not math.isfinite(float(value)) or float(value) <= 0 for value in metric_bounds)
            or float(metric_bounds[0]) > float(metric_bounds[1])
        ):
            raise ValueError("metric_eigenvalue_bounds must be positive finite (min, max)")
        if metric_function is not None and not callable(metric_function):
            raise TypeError("metric_tensor must be callable")
        if type(metric_is_constant) is not bool:
            raise TypeError("metric_is_constant must be a bool")

    goal_value = getattr(goal_region, "state", None)
    has_goal_point = goal_value is not None
    goal_state = _copy_state(goal_value, "goal", dimension) if has_goal_point else None
    if planner in {
        "informed_rrt_star",
        "fmt_star",
        "bit_star",
        "abit_star",
        "ait_star",
        "eit_star",
        "fcit_star",
        "rit_star",
        "jit_star",
        "rrt_connect",
    }:
        from pathplanning.planners.sampling._internal.continuous import exact_goal_state

        goal_state = _copy_state(exact_goal_state(goal_region, dim=dimension), "goal", dimension)
        has_goal_point = True

    from pathplanning.core.contracts import GoalState

    native_goal = False
    goal_radius = 0.0
    if isinstance(goal_region, GoalState) and has_goal_point:
        if goal_region.distance_fn is None and goal_region.radius == 0.0:
            native_goal = True
        elif owned_space is not None and goal_region.distance_fn == space.distance:
            native_goal = True
            goal_radius = float(goal_region.radius)

    requires_python_callbacks = (
        owned_space is None
        or not native_goal
        or objective is not None
        or metric_function is not None
        or manipulability_function is not None
    )
    if requires_python_callbacks and not parameters.allow_python_callbacks:
        reasons = []
        if owned_space is None:
            reasons.append(
                "space has no native model; implement to_native_model() or use a built-in space"
            )
        if not native_goal:
            reasons.append("goal requires a Python predicate or distance callback")
        if objective is not None:
            reasons.append("objective requires a Python callback")
        if metric_function is not None:
            reasons.append("metric tensor requires a Python callback")
        if manipulability_function is not None:
            reasons.append("robot Jacobian requires a Python callback")
        parameter_name = (
            "RitParams"
            if planner == "rit_star"
            else "JitParams"
            if planner == "jit_star"
            else "RrtParams"
        )
        raise ValueError(
            "Python callbacks are disabled for native planning ("
            + "; ".join(reasons)
            + f"). Set {parameter_name}(allow_python_callbacks=True) to opt in."
        )

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

    def metric_tensor(_user_data, point, dim, out_tensor):
        def call():
            if not callable(metric_function):
                raise RuntimeError("metric_tensor callback is unavailable")
            width = int(dim)
            tensor = np.asarray(metric_function(_array(point, width).copy()), dtype=np.float64)
            if tensor.shape != (width, width) or not np.all(np.isfinite(tensor)):
                raise ValueError("metric_tensor must return a finite square matrix")
            np.copyto(
                np.ctypeslib.as_array(out_tensor, shape=(width * width,)),
                tensor.reshape(-1),
            )
            return 0

        return guarded(call, -1)

    def manipulability(_user_data, point, dim, out_sigma_min):
        def call():
            if not callable(manipulability_function):
                raise RuntimeError("Jacobian callback is unavailable")
            width = int(dim)
            jacobian = np.asarray(
                manipulability_function(_array(point, width).copy()), dtype=np.float64
            )
            if (
                jacobian.ndim != 2
                or jacobian.shape[0] == 0
                or jacobian.shape[1] != width
                or not np.all(np.isfinite(jacobian))
            ):
                raise ValueError(
                    "space.jacobian(state) must be a finite matrix with one column per joint"
                )
            singular_values = np.linalg.svd(jacobian, compute_uv=False)
            if singular_values.size == 0 or not np.isfinite(singular_values[-1]):
                raise ValueError("space.jacobian(state) produced no finite singular values")
            out_sigma_min[0] = float(singular_values[-1])
            return 0

        return guarded(call, -1)

    def is_goal(_user_data, point, dim):
        return guarded(lambda: int(bool(goal_region.contains(_array(point, int(dim)).copy()))), -1)

    def goal_distance(_user_data, point, dim):
        def call():
            state = _array(point, int(dim)).copy()
            method = getattr(goal_region, "distance_to_goal", None)
            if callable(method):
                return float(method(state))
            if goal_state is not None:
                return float(space.distance(state, goal_state))
            return math.inf

        return guarded(call, math.nan)

    def path_objective(_user_data, path_pointer, path_length, dim):
        def call():
            length, width = int(path_length), int(dim)
            points = (
                np.ctypeslib.as_array(path_pointer, shape=(length * width,))
                .copy()
                .reshape(length, width)
            )
            return float(objective.path_cost(tuple(points), space))

        return guarded(call, math.nan)

    callback_objects = (
        _Sample(sample) if owned_space is None else _Sample(),
        _StateValid(state_valid) if owned_space is None else _StateValid(),
        _MotionValid(motion_valid) if owned_space is None else _MotionValid(),
        _batch_motion_callback(space, callback_errors)
        if owned_space is None
        else _MotionValidBatch(),
        _MetricTensor(metric_tensor) if callable(metric_function) else _MetricTensor(),
        _Manipulability(manipulability) if callable(manipulability_function) else _Manipulability(),
        _Distance(distance) if owned_space is None else _Distance(),
        _Steer(steer) if owned_space is None else _Steer(),
        _Goal(is_goal) if not native_goal else _Goal(),
        _GoalDistance(goal_distance) if not native_goal else _GoalDistance(),
        _Objective(path_objective) if objective is not None else _Objective(),
    )
    native_model_pointer = (
        ctypes.pointer(owned_space.model)
        if owned_space is not None
        else ctypes.POINTER(_SpaceModel)()
    )
    callbacks = _Callbacks(
        None,
        *callback_objects,
        native_model_pointer,
        int(native_goal),
        goal_radius,
    )
    seed = int(rng.integers(0, np.iinfo(np.uint64).max, dtype=np.uint64))
    euclidean_index = (
        owned_space is not None or getattr(space, "distance_metric", None) == "euclidean"
    )
    if planner in {
        "informed_rrt_star",
        "fmt_star",
        "bit_star",
        "abit_star",
        "ait_star",
        "eit_star",
        "fcit_star",
        "jit_star",
    }:
        from pathplanning.planners.sampling._internal.continuous import euclidean_distance

        euclidean_distance(space, start_state, goal_state)
        euclidean_index = True

    options = _Options(
        _ALGORITHMS[planner],
        parameters.max_iters,
        parameters.sample_count,
        parameters.batch_size,
        parameters.max_sample_tries,
        seed,
        parameters.step_size,
        getattr(parameters, "goal_sample_rate", 0.0),
        parameters.collision_step,
        getattr(parameters, "goal_reach_tolerance", 1e-9),
        getattr(parameters, "rrt_star_radius_gamma", getattr(parameters, "gamma", 2.0)),
        getattr(
            parameters,
            "rrt_star_radius_max_factor",
            getattr(parameters, "max_connection_radius", 6.0),
        ),
        getattr(parameters, "abit_inflation_parameter", 0.0),
        getattr(parameters, "abit_truncation_parameter", 0.0),
        0.0 if parameters.time_budget_s is None else parameters.time_budget_s,
        int(euclidean_index),
        float(metric_bounds[0]),
        float(metric_bounds[1]),
        int(metric_is_constant),
        int(getattr(parameters, "quadrature_order", 10)),
        int(getattr(parameters, "carm_update_interval", 15)),
        float(getattr(parameters, "carm_sigma", 0.1)),
        float(getattr(parameters, "carm_alpha", 10.0)),
        int(getattr(parameters, "jit_ancestor_depth", 8)),
        int(getattr(parameters, "jit_sample_count", 4)),
        float(getattr(parameters, "jit_sample_radius", 0.25)),
        float(getattr(parameters, "jit_bias_probability", 0.35)),
        float(getattr(parameters, "manipulability_weight", 0.0)),
        float(getattr(parameters, "manipulability_eta", 0.1)),
        float(getattr(parameters, "manipulability_epsilon", 1e-6)),
    )
    library = load_continuous_library() if trace is None else load_continuous_trace_library()
    library.pp_continuous_plan.argtypes = [
        ctypes.POINTER(_Callbacks),
        _Point,
        _Point,
        ctypes.c_size_t,
        ctypes.c_int,
        ctypes.POINTER(_Options),
        ctypes.POINTER(_Result),
    ]
    library.pp_continuous_plan.restype = ctypes.c_int
    library.pp_continuous_free_result.argtypes = [ctypes.POINTER(_Result)]
    library.pp_continuous_free_result.restype = None
    if trace is not None:
        library.pp_continuous_plan_traced.argtypes = [
            *library.pp_continuous_plan.argtypes,
            ctypes.c_uint64,
            ctypes.POINTER(TraceResult),
        ]
        library.pp_continuous_plan_traced.restype = ctypes.c_int
        library.pp_continuous_trace_free_result.argtypes = [ctypes.POINTER(TraceResult)]
        library.pp_continuous_trace_free_result.restype = None
    native_start = start_state.ctypes.data_as(_Point)
    native_goal = goal_state.ctypes.data_as(_Point) if goal_state is not None else _Point()
    native_result = _Result()
    trace_result = TraceResult()
    call_args = (
        ctypes.byref(callbacks),
        native_start,
        native_goal,
        dimension,
        int(has_goal_point),
        ctypes.byref(options),
        ctypes.byref(native_result),
    )
    status = (
        library.pp_continuous_plan(*call_args)
        if trace is None
        else library.pp_continuous_plan_traced(
            *call_args, ctypes.c_uint64(trace.max_bytes), ctypes.byref(trace_result)
        )
    )
    try:
        if callback_errors:
            raise callback_errors[0]
        if status != 0 or native_result.stop_reason == 4:
            message = (
                native_result.error_message.decode("utf-8", errors="replace")
                if native_result.error_message
                else "native continuous planner failed"
            )
            raise RuntimeError(message)
        path = None
        if native_result.success and native_result.path:
            flat_path = np.ctypeslib.as_array(
                native_result.path,
                shape=(int(native_result.path_length) * dimension,),
            ).copy()
            path = flat_path.reshape(int(native_result.path_length), dimension)
        stats = {
            "elapsed_s": float(native_result.elapsed_s),
            "sample_count": float(native_result.sample_count),
            "batches": float(native_result.batches),
            "motion_checks": float(native_result.motion_checks),
            "rewires": float(native_result.rewires),
            "metric_evaluations": float(native_result.metric_evaluations),
            "metric_updates": float(native_result.metric_updates),
            "jit_biased_samples": float(native_result.jit_biased_samples),
            "jit_ancestor_candidates": float(native_result.jit_ancestor_candidates),
            "manipulability_evaluations": float(native_result.manipulability_evaluations),
            "python_callbacks": float(requires_python_callbacks),
            "native_space_model": float(owned_space is not None),
        }
        if native_result.success:
            stats["path_cost"] = float(native_result.path_cost)
            if objective is not None:
                stats["objective_cost"] = float(native_result.path_cost)
        if parameters.time_budget_s is not None:
            stats["time_budget_s"] = float(parameters.time_budget_s)
        planner_trace = (
            copy_trace_result(trace_result, kind="continuous") if trace is not None else None
        )
        return PlanResult(
            success=bool(native_result.success),
            path=path,
            best_path=path if native_result.success else None,
            stop_reason=_STOP_REASONS.get(int(native_result.stop_reason), StopReason.NO_PROGRESS),
            iters=int(native_result.iters),
            nodes=int(native_result.nodes),
            stats=stats,
            trace=planner_trace,
        )
    finally:
        library.pp_continuous_free_result(ctypes.byref(native_result))
        if trace is not None:
            library.pp_continuous_trace_free_result(ctypes.byref(trace_result))


def run_native_dynamic_rrt(*args: Any, **kwargs: Any):
    """Run one compatibility dynamic RRT call through its private adapter."""
    from pathplanning.native.continuous_dynamic import run_native_dynamic_rrt as run

    return run(*args, **kwargs)


__all__ = ["run_native_continuous", "run_native_dynamic_rrt"]
