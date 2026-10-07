"""Jump Point Search on uniform-cost occupancy grids."""

from __future__ import annotations

from collections.abc import Mapping
import ctypes
import time

import numpy as np

from pathplanning.core.contracts import DiscreteProblem
from pathplanning.core.results import PlanResult, StopReason
from pathplanning.core.trace import TraceOptions
from pathplanning.core.types import RNG
from pathplanning.native._ffi import (
    JpsMetrics,
    SearchResult,
    TraceResult,
    copy_trace_result,
    load_native_library,
    load_search_trace_library,
)
from pathplanning.planners.search._grid_utils import Cell, _grid, _max_expansions
from pathplanning.spaces.grid2d import Grid2DSearchSpace


def plan_jps(
    problem: DiscreteProblem[Cell],
    *,
    params: Mapping[str, object] | None = None,
    rng: RNG | None = None,
    trace: TraceOptions | None = None,
) -> PlanResult:
    """Run native, optimal JPS on a uniform-cost 8-connected grid."""
    total_started = time.perf_counter()
    del rng
    graph = _grid(problem)
    if hasattr(problem.goal, "is_goal"):
        raise TypeError("JPS requires an exact grid-cell goal")
    if not isinstance(problem.goal, (tuple, list)):
        raise TypeError("JPS requires an exact grid-cell goal")
    if getattr(graph.edge_cost, "__func__", None) is not Grid2DSearchSpace.edge_cost:
        raise ValueError("JPS requires the uniform Grid2DSearchSpace edge-cost model")
    start = graph._coerce_cell(problem.start)
    goal = graph._coerce_cell(problem.goal)
    max_expansions = _max_expansions(params)

    width, height = graph.x_range, graph.y_range
    snapshot_started = time.perf_counter()
    valid_nodes = np.fromiter(
        (graph.is_valid_node((x, y)) for y in range(height) for x in range(width)),
        dtype=np.uint8,
        count=width * height,
    )
    grid_snapshot_s = time.perf_counter() - snapshot_started
    native_result = SearchResult()
    native_metrics = JpsMetrics()
    native_trace = TraceResult() if trace is not None else None
    trace_max_bytes = 0 if trace is None else trace.max_bytes
    library = load_search_trace_library() if trace is not None else load_native_library()
    valid_pointer = valid_nodes.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8))
    native_started = time.perf_counter()
    if native_trace is None:
        return_code = library.pp_native_jps_grid(
            width,
            height,
            valid_pointer,
            start[0],
            start[1],
            goal[0],
            goal[1],
            int(max_expansions is not None),
            0 if max_expansions is None else max_expansions,
            ctypes.byref(native_result),
            ctypes.byref(native_metrics),
        )
    else:
        return_code = library.pp_native_jps_grid_traced(
            width,
            height,
            valid_pointer,
            start[0],
            start[1],
            goal[0],
            goal[1],
            int(max_expansions is not None),
            0 if max_expansions is None else max_expansions,
            trace_max_bytes,
            ctypes.byref(native_result),
            ctypes.byref(native_trace),
            ctypes.byref(native_metrics),
        )
    native_search_s = time.perf_counter() - native_started
    try:
        if return_code != 0:
            message = native_result.error_message
            detail = "unknown native JPS error" if message is None else message.decode("utf-8")
            raise RuntimeError(detail)
        stop_reasons = {
            0: StopReason.SUCCESS,
            1: StopReason.MAX_ITERS,
            2: StopReason.NO_PROGRESS,
            4: StopReason.TIME_BUDGET,
        }
        try:
            reason = stop_reasons[native_result.stop_reason]
        except KeyError as exc:
            raise RuntimeError("native JPS returned an unknown stop reason") from exc
        path = None
        if native_result.success:
            path_ids = np.ctypeslib.as_array(
                native_result.path_ids,
                shape=(native_result.path_length,),
            ).copy()
            path = np.column_stack((path_ids % width, path_ids // width)).astype(float)
        expanded = int(native_result.iters)
        planner_trace = None
        trace_graph_bytes = 0
        if native_trace is not None:
            trace_graph_bytes = int(valid_nodes.nbytes)
            labels = tuple((node % width, node // width) for node in range(width * height))
            planner_trace = copy_trace_result(
                native_trace,
                kind="discrete",
                node_labels=labels,
                graph_bytes=trace_graph_bytes,
            )
        return PlanResult(
            success=bool(native_result.success),
            path=path,
            best_path=path,
            stop_reason=reason,
            iters=expanded,
            nodes=int(native_result.nodes),
            stats={
                "expanded": float(expanded),
                "grid_snapshot_s": grid_snapshot_s,
                "jps_expanded": float(expanded),
                "motion_checks": float(native_metrics.motion_checks),
                "native_search_s": native_search_s,
                "path_cost": float(native_result.path_cost),
                "planner_total_s": time.perf_counter() - total_started,
                **({"trace_graph_bytes": float(trace_graph_bytes)} if planner_trace else {}),
            },
            trace=planner_trace,
        )
    finally:
        library.pp_search_free_result(ctypes.byref(native_result))
        if native_trace is not None:
            library.pp_search_trace_free_result(ctypes.byref(native_trace))


__all__ = ["plan_jps"]
