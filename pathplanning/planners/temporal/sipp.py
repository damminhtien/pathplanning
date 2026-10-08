"""Safe Interval Path Planning for graphs with temporal obstacles."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
import ctypes
import math
import time
from typing import Any, TypeVar

import numpy as np

from pathplanning.core.contracts import DiscreteProblem, TemporalProblem, TimeInterval
from pathplanning.core.results import StopReason, TemporalPlanResult
from pathplanning.core.trace import TraceOptions
from pathplanning.core.types import RNG
from pathplanning.native import NativeGraph
from pathplanning.native._ffi import (
    GraphCsrView,
    SippResult,
    TraceResult,
    copy_trace_result,
    load_native_library,
    load_search_trace_library,
)
from pathplanning.planners.search._grid_utils import _max_expansions
from pathplanning.planners.search._internal.native import _ProblemAdapter

N = TypeVar("N")


def _intervals(value: Sequence[TimeInterval] | None, *, name: str) -> list[TimeInterval]:
    if value is None:
        return []
    normalized: list[tuple[float, float]] = []
    for raw_interval in value:
        if isinstance(raw_interval, (str, bytes)) or len(raw_interval) != 2:
            raise ValueError(f"{name} entries must be [start, end) pairs")
        start_raw, end_raw = raw_interval
        if (
            isinstance(start_raw, bool)
            or isinstance(end_raw, bool)
            or not all(
                isinstance(bound, (int, float, np.integer, np.floating))
                for bound in (start_raw, end_raw)
            )
        ):
            raise TypeError(f"{name} interval bounds must be numbers")
        start, end = float(start_raw), float(end_raw)
        if math.isnan(start) or math.isnan(end) or start >= end:
            raise ValueError(f"{name} intervals must have start < end and no NaN bounds")
        normalized.append((start, end))
    normalized.sort()
    merged: list[tuple[float, float]] = []
    for start, end in normalized:
        if merged and start <= merged[-1][1]:
            merged[-1] = (merged[-1][0], max(merged[-1][1], end))
        else:
            merged.append((start, end))
    return merged


def _safe_intervals(
    start_time: float,
    horizon: float,
    blocked: Sequence[TimeInterval],
) -> list[TimeInterval]:
    safe: list[TimeInterval] = []
    cursor = start_time
    for blocked_start, blocked_end in blocked:
        if blocked_end <= cursor:
            continue
        if blocked_start > cursor:
            safe.append((cursor, min(blocked_start, horizon)))
        cursor = max(cursor, blocked_end)
        if cursor >= horizon:
            break
    if cursor < horizon:
        safe.append((cursor, horizon))
    return safe


def _as_array(values: Sequence[float] | Sequence[int], dtype: np.dtype[Any]) -> np.ndarray:
    return np.ascontiguousarray(values, dtype=dtype)


def _pointer(array: np.ndarray, ctype: type[Any]) -> Any:
    if array.size == 0:
        return ctypes.POINTER(ctype)()
    return array.ctypes.data_as(ctypes.POINTER(ctype))


def _path_matrix(states: Sequence[N]) -> np.ndarray | None:
    try:
        path = np.asarray(states, dtype=float)
    except (TypeError, ValueError):
        return None
    return path.reshape(-1, 1) if path.ndim == 1 else path


def _runtime_limit(params: Mapping[str, object] | None) -> float:
    value = 0.0 if params is None else params.get("max_runtime_ms", 0.0)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError("max_runtime_ms must be a finite non-negative number")
    limit = float(value)
    if not math.isfinite(limit) or limit < 0.0:
        raise ValueError("max_runtime_ms must be finite and non-negative")
    return limit


def plan_sipp(
    problem: TemporalProblem[N],
    *,
    params: Mapping[str, object] | None = None,
    rng: RNG | None = None,
    trace: TraceOptions | None = None,
) -> TemporalPlanResult[N]:
    """Find the earliest-arrival path using safe node intervals and edge checks."""
    del rng
    total_started = time.perf_counter()
    if trace is not None and type(trace) is not TraceOptions:
        raise TypeError("trace must be TraceOptions or None")
    merged_params = dict(problem.params or {})
    if params is not None:
        merged_params.update(params)
    max_expansions = _max_expansions(merged_params)
    max_runtime_ms = _runtime_limit(merged_params)
    if isinstance(problem.start_time, bool) or not isinstance(
        problem.start_time, (int, float, np.integer, np.floating)
    ):
        raise TypeError("start_time must be finite")
    start_time = float(problem.start_time)
    if not math.isfinite(start_time):
        raise ValueError("start_time must be finite")
    if isinstance(problem.horizon, bool) or (
        problem.horizon is not None
        and not isinstance(problem.horizon, (int, float, np.integer, np.floating))
    ):
        raise TypeError("horizon must be a number")
    horizon = math.inf if problem.horizon is None else float(problem.horizon)
    if math.isnan(horizon) or horizon <= start_time:
        raise ValueError("horizon must be greater than start_time")

    graph_started = time.perf_counter()
    if isinstance(problem.graph, NativeGraph):
        native_graph = problem.graph
    else:
        discrete_problem = DiscreteProblem(
            problem.graph,
            problem.start,
            problem.goal,
            params=merged_params or None,
        )
        native_graph = _ProblemAdapter(discrete_problem, use_heuristic=False).native_graph
    start_id = native_graph._node_id(problem.start)
    goal_id = native_graph._node_id(problem.goal)
    if start_id is None or goal_id is None:
        raise ValueError("SIPP start and goal must be present in the graph")

    library = load_search_trace_library() if trace is not None else load_native_library()
    view = GraphCsrView()
    if library.pp_graph_export_csr_view(native_graph._native_handle, ctypes.byref(view)) != 0:
        raise RuntimeError("could not export graph adjacency for SIPP")

    node_blocked: dict[int, list[TimeInterval]] = {}
    for node, intervals in (problem.node_blocked or {}).items():
        node_id = native_graph._node_id(node)
        if node_id is None:
            raise ValueError(f"node constraint references an unknown graph node: {node!r}")
        node_blocked[node_id] = _intervals(intervals, name="node_blocked")

    safe_offsets = [0]
    safe_starts: list[float] = []
    safe_ends: list[float] = []
    for node_id in range(int(view.node_count)):
        for interval_start, interval_end in _safe_intervals(
            start_time,
            horizon,
            node_blocked.get(node_id, ()),
        ):
            safe_starts.append(interval_start)
            safe_ends.append(interval_end)
        safe_offsets.append(len(safe_starts))

    edge_blocked: dict[tuple[int, int], list[TimeInterval]] = {}
    for edge, intervals in (problem.edge_blocked or {}).items():
        if len(edge) != 2:
            raise ValueError("edge_blocked keys must be directed (source, target) pairs")
        source_id = native_graph._node_id(edge[0])
        target_id = native_graph._node_id(edge[1])
        if source_id is None or target_id is None:
            raise ValueError(f"edge constraint references an unknown graph node: {edge!r}")
        edge_blocked[(source_id, target_id)] = _intervals(intervals, name="edge_blocked")

    durations: dict[tuple[int, int], float] = {}
    duration_keys: set[tuple[int, int]] = set()
    for edge, duration in (problem.edge_durations or {}).items():
        if len(edge) != 2:
            raise ValueError("edge_durations keys must be directed (source, target) pairs")
        source_id = native_graph._node_id(edge[0])
        target_id = native_graph._node_id(edge[1])
        if source_id is None or target_id is None:
            raise ValueError(f"duration references an unknown graph node: {edge!r}")
        if isinstance(duration, bool) or not isinstance(
            duration, (int, float, np.integer, np.floating)
        ):
            raise TypeError("edge durations must be positive finite numbers")
        value = float(duration)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError("edge durations must be positive finite numbers")
        durations[(source_id, target_id)] = value
        duration_keys.add((source_id, target_id))

    edge_block_offsets = [0]
    edge_block_starts: list[float] = []
    edge_block_ends: list[float] = []
    edge_durations: list[float] = []
    found_duration_keys: set[tuple[int, int]] = set()
    found_edge_keys: set[tuple[int, int]] = set()
    for source_id in range(int(view.node_count)):
        for edge_id in range(int(view.offsets[source_id]), int(view.offsets[source_id + 1])):
            target_id = int(view.neighbor_ids[edge_id])
            key = (source_id, target_id)
            found_edge_keys.add(key)
            base_duration = float(view.edge_costs[edge_id])
            duration = durations.get(key, base_duration)
            if not math.isfinite(duration) or duration <= 0.0:
                raise ValueError(f"edge {key!r} needs a positive finite duration")
            edge_durations.append(duration)
            intervals = edge_blocked.get(key, ())
            for interval_start, interval_end in intervals:
                edge_block_starts.append(interval_start)
                edge_block_ends.append(interval_end)
            edge_block_offsets.append(len(edge_block_starts))
            if key in durations:
                found_duration_keys.add(key)
    if duration_keys - found_duration_keys:
        raise ValueError("edge_durations references an edge that is not in the graph")
    if set(edge_blocked) - found_edge_keys:
        raise ValueError("edge_blocked references an edge that is not in the graph")

    graph_init_s = time.perf_counter() - graph_started
    native_offsets = _as_array(safe_offsets, np.dtype(np.uint64))
    native_safe_starts = _as_array(safe_starts, np.dtype(np.float64))
    native_safe_ends = _as_array(safe_ends, np.dtype(np.float64))
    native_edge_block_offsets = _as_array(edge_block_offsets, np.dtype(np.uint64))
    native_edge_block_starts = _as_array(edge_block_starts, np.dtype(np.float64))
    native_edge_block_ends = _as_array(edge_block_ends, np.dtype(np.float64))
    native_durations = _as_array(edge_durations, np.dtype(np.float64))
    native_result = SippResult()
    native_trace = TraceResult() if trace is not None else None
    native_started = time.perf_counter()
    if native_trace is None:
        status = library.pp_sipp_plan(
            view.node_count,
            view.edge_count,
            view.offsets,
            view.neighbor_ids,
            _pointer(native_durations, ctypes.c_double),
            _pointer(native_offsets, ctypes.c_uint64),
            _pointer(native_safe_starts, ctypes.c_double),
            _pointer(native_safe_ends, ctypes.c_double),
            len(safe_starts),
            _pointer(native_edge_block_offsets, ctypes.c_uint64),
            _pointer(native_edge_block_starts, ctypes.c_double),
            _pointer(native_edge_block_ends, ctypes.c_double),
            len(edge_block_starts),
            start_id,
            goal_id,
            start_time,
            int(max_expansions is not None),
            max_expansions or 0,
            max_runtime_ms,
            ctypes.byref(native_result),
        )
    else:
        assert trace is not None and native_trace is not None
        status = library.pp_sipp_plan_traced(
            view.node_count,
            view.edge_count,
            view.offsets,
            view.neighbor_ids,
            _pointer(native_durations, ctypes.c_double),
            _pointer(native_offsets, ctypes.c_uint64),
            _pointer(native_safe_starts, ctypes.c_double),
            _pointer(native_safe_ends, ctypes.c_double),
            len(safe_starts),
            _pointer(native_edge_block_offsets, ctypes.c_uint64),
            _pointer(native_edge_block_starts, ctypes.c_double),
            _pointer(native_edge_block_ends, ctypes.c_double),
            len(edge_block_starts),
            start_id,
            goal_id,
            start_time,
            int(max_expansions is not None),
            max_expansions or 0,
            max_runtime_ms,
            trace.max_bytes,
            ctypes.byref(native_result),
            ctypes.byref(native_trace),
        )
    native_search_s = time.perf_counter() - native_started
    try:
        if status != 0:
            detail = "unknown native SIPP error"
            if native_result.error_message is not None:
                detail = native_result.error_message.decode("utf-8", errors="replace")
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
            raise RuntimeError("native SIPP returned an unknown stop reason") from exc
        states: tuple[N, ...] = ()
        times: tuple[float, ...] = ()
        if native_result.success:
            path_ids = np.ctypeslib.as_array(
                native_result.path_ids,
                shape=(native_result.path_length,),
            )
            arrival_times = np.ctypeslib.as_array(
                native_result.arrival_times,
                shape=(native_result.path_length,),
            )
            states = tuple(native_graph.node_labels[int(node_id)] for node_id in path_ids)
            times = tuple(float(value) for value in arrival_times)
        planner_trace = None
        trace_graph_bytes = 0
        if native_trace is not None:
            trace_graph_bytes = int(
                (int(view.node_count) + 1) * 8
                + int(view.edge_count) * 16
                + native_offsets.nbytes
                + native_safe_starts.nbytes
                + native_safe_ends.nbytes
                + native_edge_block_offsets.nbytes
                + native_edge_block_starts.nbytes
                + native_edge_block_ends.nbytes
                + native_durations.nbytes
            )
            planner_trace = copy_trace_result(
                native_trace,
                kind="discrete",
                node_labels=native_graph.node_labels,
                graph_bytes=trace_graph_bytes,
            )
        success = bool(native_result.success)
        return TemporalPlanResult(
            success=success,
            path=_path_matrix(states) if success else None,
            best_path=_path_matrix(states) if success else None,
            stop_reason=stop_reason,
            iters=int(native_result.iters),
            nodes=int(native_result.nodes),
            stats={
                "arrival_time": float(native_result.arrival_time),
                "expanded": float(native_result.iters),
                "graph_init_s": graph_init_s,
                "native_search_s": native_search_s,
                "path_cost": float(native_result.path_cost),
                "planner_total_s": time.perf_counter() - total_started,
                **(
                    {"trace_graph_bytes": float(trace_graph_bytes)}
                    if planner_trace is not None
                    else {}
                ),
            },
            trace=planner_trace,
            states=states,
            times=times,
        )
    finally:
        library.pp_sipp_free_result(ctypes.byref(native_result))
        if native_trace is not None:
            library.pp_search_trace_free_result(ctypes.byref(native_trace))


__all__ = ["plan_sipp"]
