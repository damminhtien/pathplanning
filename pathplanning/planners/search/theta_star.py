"""Any-angle Theta* search on 8-connected grid maps."""

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
    SearchResult,
    ThetaMetrics,
    TraceResult,
    copy_trace_result,
    load_native_library,
    load_search_trace_library,
)
from pathplanning.planners.search._grid_utils import Cell, _grid, _max_expansions
from pathplanning.spaces.grid2d import Grid2DSearchSpace
from pathplanning.spaces.terrain_grid2d import TerrainCostGrid2D


def plan_theta_star(
    problem: DiscreteProblem[Cell],
    *,
    params: Mapping[str, object] | None = None,
    rng: RNG | None = None,
    trace: TraceOptions | None = None,
) -> PlanResult:
    """Run native Theta* and return grid-cell waypoints along any-angle segments."""
    return _plan_any_angle(problem, "theta_star", "Theta*", params=params, rng=rng, trace=trace)


def _plan_any_angle(
    problem: DiscreteProblem[Cell],
    planner_id: str,
    planner_name: str,
    *,
    params: Mapping[str, object] | None = None,
    rng: RNG | None = None,
    trace: TraceOptions | None = None,
) -> PlanResult:
    """Run one native any-angle grid search implementation."""
    total_started = time.perf_counter()
    del rng
    graph = _grid(problem)
    if hasattr(problem.goal, "is_goal") or not isinstance(problem.goal, (tuple, list)):
        raise TypeError(f"{planner_name} requires an exact grid-cell goal")
    if isinstance(graph, TerrainCostGrid2D):
        if getattr(graph.edge_cost, "__func__", None) is not TerrainCostGrid2D.edge_cost:
            raise ValueError(f"{planner_name} requires the built-in terrain grid cost model")
        terrain_source = graph.terrain_costs
    else:
        if getattr(graph.edge_cost, "__func__", None) is not Grid2DSearchSpace.edge_cost:
            raise ValueError(f"{planner_name} requires a built-in uniform or terrain grid cost model")
        terrain_source = None

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
    if terrain_source is None:
        terrain_costs = np.ones(width * height, dtype=np.float64)
    else:
        terrain_costs = np.ascontiguousarray(terrain_source.reshape(-1), dtype=np.float64)
    grid_snapshot_s = time.perf_counter() - snapshot_started

    native_result = SearchResult()
    native_metrics = ThetaMetrics()
    native_trace = TraceResult() if trace is not None else None
    trace_max_bytes = 0 if trace is None else trace.max_bytes
    library = load_search_trace_library() if trace is not None else load_native_library()
    native_symbol = f"pp_native_{planner_id}_grid"
    native_function = getattr(library, native_symbol)
    valid_pointer = valid_nodes.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8))
    terrain_pointer = terrain_costs.ctypes.data_as(ctypes.POINTER(ctypes.c_double))
    native_started = time.perf_counter()
    if native_trace is None:
        return_code = native_function(
            width,
            height,
            valid_pointer,
            terrain_pointer,
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
        return_code = getattr(library, f"{native_symbol}_traced")(
            width,
            height,
            valid_pointer,
            terrain_pointer,
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
            detail = f"unknown native {planner_name} error" if message is None else message.decode("utf-8")
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
            raise RuntimeError(f"native {planner_name} returned an unknown stop reason") from exc

        path = None
        if native_result.success:
            path_ids = np.ctypeslib.as_array(
                native_result.path_ids,
                shape=(native_result.path_length,),
            ).copy()
            path = np.column_stack((path_ids % width, path_ids // width)).astype(float)
        planner_trace = None
        trace_graph_bytes = 0
        if native_trace is not None:
            trace_graph_bytes = int(valid_nodes.nbytes + terrain_costs.nbytes)
            labels = tuple((node % width, node // width) for node in range(width * height))
            planner_trace = copy_trace_result(
                native_trace,
                kind="discrete",
                node_labels=labels,
                graph_bytes=trace_graph_bytes,
            )
        expanded = int(native_result.iters)
        return PlanResult(
            success=bool(native_result.success),
            path=path,
            best_path=path,
            stop_reason=reason,
            iters=expanded,
            nodes=int(native_result.nodes),
            stats={
                "cells_checked": float(native_metrics.cells_checked),
                "expanded": float(expanded),
                "grid_snapshot_s": grid_snapshot_s,
                "line_of_sight_checks": float(native_metrics.line_of_sight_checks),
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
        )
    finally:
        library.pp_search_free_result(ctypes.byref(native_result))
        if native_trace is not None:
            library.pp_search_trace_free_result(ctypes.byref(native_trace))


__all__ = ["plan_theta_star"]
