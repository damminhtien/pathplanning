"""Thin Python binding for the native C sampling planner engine."""

from __future__ import annotations

import ctypes
import math
import time
from typing import Any

import numpy as np

from pathplanning.core.contracts import ContinuousSpace, GoalRegion, Objective, State
from pathplanning.core.params import RoadmapParams, RrtParams
from pathplanning.core.results import PlanResult, StopReason
from pathplanning.core.trace import PlannerTrace, TraceOptions
from pathplanning.core.types import RNG
from pathplanning.native._ffi import (
    TraceResult,
    copy_trace_result,
    load_continuous_library,
    load_continuous_trace_library,
)
from pathplanning.native.continuous_model import NativeContinuousSpaceModel


class _Callbacks(ctypes.Structure):
    pass


_Point = ctypes.POINTER(ctypes.c_double)
_Sample = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_void_p, _Point, ctypes.c_size_t)
_StateValid = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_void_p, _Point, ctypes.c_size_t)
_MotionValid = ctypes.CFUNCTYPE(
    ctypes.c_int, ctypes.c_void_p, _Point, _Point, ctypes.c_size_t, ctypes.c_double
)
_Distance = ctypes.CFUNCTYPE(ctypes.c_double, ctypes.c_void_p, _Point, _Point, ctypes.c_size_t)
_Steer = ctypes.CFUNCTYPE(
    ctypes.c_int, ctypes.c_void_p, _Point, _Point, ctypes.c_size_t, ctypes.c_double, _Point
)
_Goal = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_void_p, _Point, ctypes.c_size_t)
_GoalDistance = ctypes.CFUNCTYPE(ctypes.c_double, ctypes.c_void_p, _Point, ctypes.c_size_t)
_Objective = ctypes.CFUNCTYPE(
    ctypes.c_double, ctypes.c_void_p, _Point, ctypes.c_size_t, ctypes.c_size_t
)


class _SpaceModel(ctypes.Structure):
    _fields_ = [
        ("lower_bounds", _Point),
        ("upper_bounds", _Point),
        ("box_minima", _Point),
        ("box_maxima", _Point),
        ("box_count", ctypes.c_size_t),
        ("sphere_centers", _Point),
        ("sphere_radii", _Point),
        ("sphere_count", ctypes.c_size_t),
        ("obb_centers", _Point),
        ("obb_extents", _Point),
        ("obb_orientations", _Point),
        ("obb_count", ctypes.c_size_t),
    ]


class _OwnedSpaceModel:
    """Keep converted obstacle arrays alive for the duration of the C call."""

    def __init__(self, arrays: tuple[np.ndarray, ...], model: _SpaceModel) -> None:
        self.arrays = arrays
        self.model = model


_Callbacks._fields_ = [
    ("user_data", ctypes.c_void_p),
    ("sample_free", _Sample),
    ("state_valid", _StateValid),
    ("motion_valid", _MotionValid),
    ("distance", _Distance),
    ("steer", _Steer),
    ("is_goal", _Goal),
    ("goal_distance", _GoalDistance),
    ("path_objective", _Objective),
    ("native_space", ctypes.POINTER(_SpaceModel)),
    ("native_goal", ctypes.c_int),
    ("goal_radius", ctypes.c_double),
]


class _Options(ctypes.Structure):
    _fields_ = [
        ("algorithm", ctypes.c_int),
        ("max_iters", ctypes.c_uint64),
        ("sample_count", ctypes.c_uint64),
        ("batch_size", ctypes.c_uint64),
        ("max_sample_tries", ctypes.c_uint64),
        ("seed", ctypes.c_uint64),
        ("step_size", ctypes.c_double),
        ("goal_sample_rate", ctypes.c_double),
        ("collision_step", ctypes.c_double),
        ("goal_reach_tolerance", ctypes.c_double),
        ("rrt_star_radius_gamma", ctypes.c_double),
        ("rrt_star_radius_max_factor", ctypes.c_double),
        ("abit_inflation_parameter", ctypes.c_double),
        ("abit_truncation_parameter", ctypes.c_double),
        ("time_budget_s", ctypes.c_double),
        ("use_euclidean_index", ctypes.c_int),
    ]


class _Result(ctypes.Structure):
    _fields_ = [
        ("success", ctypes.c_int),
        ("stop_reason", ctypes.c_int),
        ("iters", ctypes.c_uint64),
        ("nodes", ctypes.c_uint64),
        ("sample_count", ctypes.c_uint64),
        ("batches", ctypes.c_uint64),
        ("motion_checks", ctypes.c_uint64),
        ("rewires", ctypes.c_uint64),
        ("path_cost", ctypes.c_double),
        ("elapsed_s", ctypes.c_double),
        ("path", _Point),
        ("path_length", ctypes.c_size_t),
        ("dimension", ctypes.c_size_t),
        ("error_message", ctypes.c_char_p),
    ]


class _DynamicResult(ctypes.Structure):
    _fields_ = [
        ("plan", _Result),
        ("tree_points", _Point),
        ("tree_parents", ctypes.POINTER(ctypes.c_uint32)),
        ("tree_count", ctypes.c_size_t),
        ("invalid_nodes", ctypes.POINTER(ctypes.c_uint32)),
        ("invalid_count", ctypes.c_size_t),
    ]


_ALGORITHMS = {
    "rrt": 1,
    "rrt_star": 2,
    "informed_rrt_star": 3,
    "fmt_star": 4,
    "bit_star": 5,
    "abit_star": 6,
    "rrt_connect": 7,
}
_STOP_REASONS = {
    0: StopReason.SUCCESS,
    1: StopReason.TIME_BUDGET,
    2: StopReason.MAX_ITERS,
    3: StopReason.NO_PROGRESS,
}


def _array(pointer: _Point, dimension: int) -> np.ndarray:
    return np.ctypeslib.as_array(pointer, shape=(dimension,))


def _copy_state(value: object, name: str, dimension: int) -> np.ndarray:
    state = np.asarray(value, dtype=np.float64)
    if state.shape != (dimension,) or not np.all(np.isfinite(state)):
        raise ValueError(f"{name} must be a finite state vector of length {dimension}")
    return np.ascontiguousarray(state)


def _pointer(values: np.ndarray) -> _Point:
    return values.ctypes.data_as(_Point) if values.size else _Point()


def _native_space_model(space: ContinuousSpace[State], dimension: int) -> _OwnedSpaceModel | None:
    """Serialize built-in or explicitly declared Euclidean spaces into C inputs."""
    from pathplanning.spaces.continuous_3d import ContinuousSpace3D
    from pathplanning.spaces.grid2d import Grid2DSamplingSpace

    boxes: list[tuple[list[float], list[float]]] = []
    spheres: list[tuple[list[float], float]] = []
    obbs: list[tuple[list[float], list[float], list[float]]] = []
    descriptor: NativeContinuousSpaceModel | None = None
    if type(space) is ContinuousSpace3D:
        if dimension != 3:
            raise ValueError("ContinuousSpace3D requires three-dimensional states")
        boxes.extend(
            (np.asarray(item.min_corner).tolist(), np.asarray(item.max_corner).tolist())
            for item in space.aabbs
        )
        spheres.extend(
            (np.asarray(item.center).tolist(), float(item.radius)) for item in space.spheres
        )
        obbs.extend(
            (
                np.asarray(item.center).tolist(),
                np.asarray(item.extents).tolist(),
                np.asarray(item.orientation).reshape(-1).tolist(),
            )
            for item in space.obbs
        )
        descriptor = NativeContinuousSpaceModel(
            lower_bounds=space.lower_bound,
            upper_bounds=space.upper_bound,
            box_minima=[item[0] for item in boxes],
            box_maxima=[item[1] for item in boxes],
            sphere_centers=[item[0] for item in spheres],
            sphere_radii=[item[1] for item in spheres],
            obb_centers=[item[0] for item in obbs],
            obb_extents=[item[1] for item in obbs],
            obb_orientations=[item[2] for item in obbs],
        )
    elif type(space) is Grid2DSamplingSpace:
        if dimension != 2:
            raise ValueError("Grid2DSamplingSpace requires two-dimensional states")
        delta = float(space.delta)
        for ox, oy, width, height in (*space.obs_boundary, *space.obs_rectangle):
            boxes.append(([ox - delta, oy - delta], [ox + width + delta, oy + height + delta]))
        spheres.extend(([cx, cy], float(radius) + delta) for cx, cy, radius in space.obs_circle)
        descriptor = NativeContinuousSpaceModel(
            lower_bounds=[space.x_range[0], space.y_range[0]],
            upper_bounds=[space.x_range[1], space.y_range[1]],
            box_minima=[item[0] for item in boxes],
            box_maxima=[item[1] for item in boxes],
            sphere_centers=[item[0] for item in spheres],
            sphere_radii=[item[1] for item in spheres],
        )
    else:
        provider = getattr(space, "to_native_model", None)
        if not callable(provider):
            return None
        descriptor = provider()
        if not isinstance(descriptor, NativeContinuousSpaceModel):
            raise TypeError("to_native_model() must return NativeContinuousSpaceModel")

    def vector(name: str, values: object) -> np.ndarray:
        result = np.asarray(values, dtype=np.float64)
        if result.shape != (dimension,) or not np.all(np.isfinite(result)):
            raise ValueError(f"native model {name} must be a finite vector of length {dimension}")
        return np.ascontiguousarray(result)

    def matrix(name: str, values: object, width: int) -> np.ndarray:
        result = np.asarray(values, dtype=np.float64)
        if result.size == 0:
            return np.empty((0, width), dtype=np.float64)
        if result.ndim != 2 or result.shape[1] != width or not np.all(np.isfinite(result)):
            raise ValueError(f"native model {name} must be a finite matrix with {width} columns")
        return np.ascontiguousarray(result)

    lower = vector("lower_bounds", descriptor.lower_bounds)
    upper = vector("upper_bounds", descriptor.upper_bounds)
    if np.any(lower >= upper):
        raise ValueError("native model lower_bounds must be smaller than upper_bounds")

    box_minima = matrix("box_minima", descriptor.box_minima, dimension)
    box_maxima = matrix("box_maxima", descriptor.box_maxima, dimension)
    if box_minima.shape != box_maxima.shape or np.any(box_minima > box_maxima):
        raise ValueError("native model box_minima and box_maxima must match and be ordered")

    sphere_centers = matrix("sphere_centers", descriptor.sphere_centers, dimension)
    sphere_radii = np.asarray(descriptor.sphere_radii, dtype=np.float64)
    if sphere_radii.size == 0:
        sphere_radii = np.empty((0,), dtype=np.float64)
    if (
        sphere_radii.ndim != 1
        or sphere_radii.shape[0] != sphere_centers.shape[0]
        or not np.all(np.isfinite(sphere_radii))
        or np.any(sphere_radii < 0.0)
    ):
        raise ValueError(
            "native model sphere_radii must match centers and be finite and non-negative"
        )
    sphere_radii = np.ascontiguousarray(sphere_radii)

    obb_centers = matrix("obb_centers", descriptor.obb_centers, 3)
    obb_extents = matrix("obb_extents", descriptor.obb_extents, 3)
    obb_orientations = matrix("obb_orientations", descriptor.obb_orientations, 9)
    if dimension != 3 and any(
        values.shape[0] for values in (obb_centers, obb_extents, obb_orientations)
    ):
        raise ValueError("native model OBB obstacles require three-dimensional states")
    if not (obb_centers.shape[0] == obb_extents.shape[0] == obb_orientations.shape[0]) or np.any(
        obb_extents <= 0.0
    ):
        raise ValueError("native model OBB arrays must match and have positive extents")

    arrays = (
        lower,
        upper,
        box_minima,
        box_maxima,
        sphere_centers,
        sphere_radii,
        obb_centers,
        obb_extents,
        obb_orientations,
    )
    model = _SpaceModel(
        _pointer(lower),
        _pointer(upper),
        _pointer(box_minima),
        _pointer(box_maxima),
        box_minima.shape[0],
        _pointer(sphere_centers),
        _pointer(sphere_radii),
        sphere_centers.shape[0],
        _pointer(obb_centers),
        _pointer(obb_extents),
        _pointer(obb_orientations),
        obb_centers.shape[0],
    )
    return _OwnedSpaceModel(arrays, model)


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
    ) -> None:
        self.space = space
        self.dimension = dimension
        self.params = params.validate()
        self.lazy = lazy
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
    params: RrtParams,
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

    goal_value = getattr(goal_region, "state", None)
    has_goal_point = goal_value is not None
    goal_state = _copy_state(goal_value, "goal", dimension) if has_goal_point else None
    if planner in {"informed_rrt_star", "fmt_star", "bit_star", "abit_star", "rrt_connect"}:
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

    requires_python_callbacks = owned_space is None or not native_goal or objective is not None
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
        raise ValueError(
            "Python callbacks are disabled for native planning ("
            + "; ".join(reasons)
            + "). Set RrtParams(allow_python_callbacks=True) to opt in."
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
    if planner in {"informed_rrt_star", "fmt_star", "bit_star", "abit_star"}:
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
        parameters.goal_sample_rate,
        parameters.collision_step,
        parameters.goal_reach_tolerance,
        parameters.rrt_star_radius_gamma,
        parameters.rrt_star_radius_max_factor,
        parameters.abit_inflation_parameter,
        parameters.abit_truncation_parameter,
        0.0 if parameters.time_budget_s is None else parameters.time_budget_s,
        int(euclidean_index),
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
    callbacks = _Callbacks(
        None,
        *callback_objects,
        native_model_pointer,
        1,
        0.0,
    )
    max_sample_tries = int(getattr(space, "max_sample_tries", 1_000))
    iteration_limit = int(config.max_iterations if max_iterations is None else max_iterations)
    if iteration_limit < 0 or iteration_limit >= np.iinfo(np.uint64).max:
        raise ValueError("dynamic RRT max_iterations is outside the supported range")
    options = _Options(
        _ALGORITHMS["rrt"],
        iteration_limit,
        1,
        1,
        max_sample_tries,
        0 if prune_only else int(rng.integers(0, np.iinfo(np.uint64).max, dtype=np.uint64)),
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
    )
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


__all__ = ["run_native_continuous", "run_native_dynamic_rrt"]
