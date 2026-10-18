"""Private ctypes layouts and conversion helpers for the continuous C ABI."""

from __future__ import annotations

from collections.abc import Callable, Sequence
import ctypes
from typing import Any, cast

import numpy as np

from pathplanning.core.contracts import ContinuousSpace, State
from pathplanning.core.results import StopReason
from pathplanning.native.continuous_model import NativeContinuousSpaceModel


class _Callbacks(ctypes.Structure):
    pass


_Point = ctypes.POINTER(ctypes.c_double)
_Sample = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_void_p, _Point, ctypes.c_size_t)
_StateValid = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_void_p, _Point, ctypes.c_size_t)
_MotionValid = ctypes.CFUNCTYPE(
    ctypes.c_int, ctypes.c_void_p, _Point, _Point, ctypes.c_size_t, ctypes.c_double
)
_MotionValidBatch = ctypes.CFUNCTYPE(
    ctypes.c_int,
    ctypes.c_void_p,
    _Point,
    _Point,
    ctypes.c_size_t,
    ctypes.c_size_t,
    ctypes.c_double,
    ctypes.POINTER(ctypes.c_uint8),
)
_MetricTensor = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_void_p, _Point, ctypes.c_size_t, _Point)
_Manipulability = ctypes.CFUNCTYPE(
    ctypes.c_int, ctypes.c_void_p, _Point, ctypes.c_size_t, ctypes.POINTER(ctypes.c_double)
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
    ("motion_valid_batch", _MotionValidBatch),
    ("metric_tensor", _MetricTensor),
    ("manipulability", _Manipulability),
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
        ("metric_lambda_min", ctypes.c_double),
        ("metric_lambda_max", ctypes.c_double),
        ("metric_is_constant", ctypes.c_int),
        ("riemannian_quadrature_order", ctypes.c_int),
        ("carm_update_interval", ctypes.c_uint64),
        ("carm_sigma", ctypes.c_double),
        ("carm_alpha", ctypes.c_double),
        ("jit_ancestor_depth", ctypes.c_uint64),
        ("jit_sample_count", ctypes.c_uint64),
        ("jit_sample_radius", ctypes.c_double),
        ("jit_bias_probability", ctypes.c_double),
        ("manipulability_weight", ctypes.c_double),
        ("manipulability_eta", ctypes.c_double),
        ("manipulability_epsilon", ctypes.c_double),
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
        ("metric_evaluations", ctypes.c_uint64),
        ("metric_updates", ctypes.c_uint64),
        ("jit_biased_samples", ctypes.c_uint64),
        ("jit_ancestor_candidates", ctypes.c_uint64),
        ("manipulability_evaluations", ctypes.c_uint64),
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


class _DynamicSessionResult(ctypes.Structure):
    _fields_ = [
        ("plan", _Result),
        ("tree_points", _Point),
        ("tree_parents", ctypes.POINTER(ctypes.c_uint32)),
        ("tree_count", ctypes.c_size_t),
        ("tree_offset", ctypes.c_size_t),
        ("tree_is_snapshot", ctypes.c_int),
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
    "ait_star": 8,
    "eit_star": 9,
    "fcit_star": 10,
    "rit_star": 11,
    "jit_star": 12,
}
_STOP_REASONS = {
    0: StopReason.SUCCESS,
    1: StopReason.TIME_BUDGET,
    2: StopReason.MAX_ITERS,
    3: StopReason.NO_PROGRESS,
}


def _array(pointer: _Point, dimension: int) -> np.ndarray:
    return np.ctypeslib.as_array(pointer, shape=(dimension,))


def _batch_motion_callback(
    space: ContinuousSpace[State], callback_errors: list[BaseException]
) -> Any:
    """Adapt the optional Python batch checker to the native callback ABI."""
    with_step = getattr(space, "is_motion_valid_batch_with_step", None)
    batch = getattr(space, "is_motion_valid_batch", None)
    if not callable(with_step) and not callable(batch):
        return _MotionValidBatch()

    def callback(_user_data, starts_pointer, ends_pointer, count, dim, step, out_validity):
        if callback_errors:
            return -1
        try:
            edge_count, width = int(count), int(dim)
            starts = (
                np.ctypeslib.as_array(starts_pointer, shape=(edge_count * width,))
                .copy()
                .reshape(edge_count, width)
            )
            ends = (
                np.ctypeslib.as_array(ends_pointer, shape=(edge_count * width,))
                .copy()
                .reshape(edge_count, width)
            )
            edges = [(starts[index], ends[index]) for index in range(edge_count)]
            if callable(with_step):
                checker = cast(
                    Callable[[list[tuple[np.ndarray, np.ndarray]], float], Sequence[object]],
                    with_step,
                )
                values = checker(edges, float(step))
            elif callable(batch):
                checker = cast(
                    Callable[[list[tuple[np.ndarray, np.ndarray]]], Sequence[object]], batch
                )
                values = checker(edges)
            else:
                raise TypeError("batch motion checker is unavailable")
            if len(values) != edge_count:
                raise ValueError("batch motion checker returned the wrong number of results")
            validity = np.ctypeslib.as_array(out_validity, shape=(edge_count,))
            for index, value in enumerate(values):
                validity[index] = int(bool(value))
            return 0
        except BaseException as exc:  # ctypes callbacks cannot propagate Python exceptions.
            callback_errors.append(exc)
            return -1

    return _MotionValidBatch(callback)


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


__all__ = [
    "_ALGORITHMS",
    "_Callbacks",
    "_Distance",
    "_DynamicResult",
    "_DynamicSessionResult",
    "_Goal",
    "_GoalDistance",
    "_Manipulability",
    "_MetricTensor",
    "_MotionValid",
    "_MotionValidBatch",
    "_Objective",
    "_Options",
    "_OwnedSpaceModel",
    "_Point",
    "_Result",
    "_STOP_REASONS",
    "_Sample",
    "_SpaceModel",
    "_StateValid",
    "_Steer",
    "_array",
    "_batch_motion_callback",
    "_copy_state",
    "_native_space_model",
    "_pointer",
]
