"""Shared graph preparation and result validation for native MAPF planners."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
import ctypes
import math
import time
from typing import Any, TypeVar, cast

import numpy as np

from pathplanning.core.contracts import MultiAgentProblem
from pathplanning.core.results import MultiAgentPlanResult, StopReason
from pathplanning.core.trace import TraceOptions
from pathplanning.core.types import RNG
from pathplanning.native import NativeGraph
from pathplanning.native._ffi import (
    GraphCsrView,
    MapfResult,
    TraceResult,
    copy_trace_result,
    load_native_library,
    load_search_trace_library,
)
from pathplanning.planners.search._grid_utils import _max_expansions

N = TypeVar("N")
_STOP_REASONS = {
    0: StopReason.SUCCESS,
    1: StopReason.MAX_ITERS,
    2: StopReason.NO_PROGRESS,
    4: StopReason.TIME_BUDGET,
}


def _materialize_graph(
    problem: MultiAgentProblem[N], params: Mapping[str, object] | None
) -> NativeGraph[N]:
    if isinstance(problem.graph, NativeGraph):
        return problem.graph
    factory = getattr(problem.graph, "to_native_graph", None)
    if callable(factory):
        graph = factory()
        if not isinstance(graph, NativeGraph):
            raise TypeError("to_native_graph() must return NativeGraph")
        return cast(NativeGraph[N], graph)

    limit_value = None if params is None else params.get("max_materialized_nodes")
    if limit_value is None:
        limit = 1_000_000
    elif isinstance(limit_value, bool) or not isinstance(limit_value, int) or limit_value <= 0:
        raise ValueError("max_materialized_nodes must be a positive integer")
    else:
        limit = limit_value

    labels: list[N] = []
    ids: dict[N, int] = {}
    rows: list[list[tuple[int, float]]] = []
    queue: list[int] = []

    def ensure_node(node: N) -> int:
        try:
            found = ids.get(node)
        except TypeError as exc:
            raise TypeError("multi-agent graph nodes must be hashable") from exc
        if found is not None:
            return found
        if len(labels) >= limit:
            raise ValueError(
                "reachable MAPF graph exceeds max_materialized_nodes; pass a NativeGraph"
            )
        node_id = len(labels)
        labels.append(node)
        ids[node] = node_id
        rows.append([])
        queue.append(node_id)
        return node_id

    for node in (*problem.starts, *problem.goals):
        ensure_node(node)
    cursor = 0
    while cursor < len(queue):
        source_id = queue[cursor]
        source = labels[source_id]
        try:
            neighbors = problem.graph.neighbors(source)
            for target in neighbors:
                target_id = ensure_node(target)
                raw_cost = problem.graph.edge_cost(source, target)
                if isinstance(raw_cost, bool) or not isinstance(raw_cost, (int, float, np.number)):
                    raise TypeError("MAPF edge costs must be unit integers or floats")
                cost = float(raw_cost)
                if not math.isfinite(cost) or abs(cost - 1.0) > 1e-12:
                    raise ValueError("MAPF requires unit-weight edges")
                rows[source_id].append((target_id, cost))
        except (KeyError, TypeError, ValueError, RuntimeError):
            raise
        cursor += 1

    offsets = [0]
    neighbor_ids: list[int] = []
    weights: list[float] = []
    for row in rows:
        for target_id, cost in row:
            neighbor_ids.append(target_id)
            weights.append(cost)
        offsets.append(len(neighbor_ids))
    return NativeGraph.from_csr(offsets, neighbor_ids, weights, node_labels=labels)


def _prepare_problem(
    problem: MultiAgentProblem[N], params: Mapping[str, object] | None
) -> tuple[NativeGraph[N], GraphCsrView, np.ndarray, np.ndarray, int]:
    starts = tuple(problem.starts)
    goals = tuple(problem.goals)
    if not starts or len(starts) != len(goals):
        raise ValueError("multi-agent starts and goals must be non-empty and have equal lengths")
    try:
        if len(set(starts)) != len(starts):
            raise ValueError("multi-agent starts must be unique")
        if len(set(goals)) != len(goals):
            raise ValueError("multi-agent goals must be unique")
    except TypeError as exc:
        raise TypeError("multi-agent starts and goals must be hashable") from exc
    is_valid_node = getattr(problem.graph, "is_valid_node", None)
    if callable(is_valid_node) and any(not is_valid_node(node) for node in (*starts, *goals)):
        raise ValueError("multi-agent starts and goals must be valid graph nodes")

    graph = _materialize_graph(problem, params)
    library = load_native_library()
    view = GraphCsrView()
    if library.pp_graph_export_csr_view(graph._native_handle, ctypes.byref(view)) != 0:
        raise RuntimeError("could not export graph adjacency for MAPF")
    edge_pairs: set[tuple[int, int]] = set()
    costs = (
        np.ctypeslib.as_array(view.edge_costs, shape=(int(view.edge_count),))
        if view.edge_count
        else np.empty(0, dtype=np.float64)
    )
    for source in range(int(view.node_count)):
        for edge_id in range(int(view.offsets[source]), int(view.offsets[source + 1])):
            target = int(view.neighbor_ids[edge_id])
            if source == target:
                raise ValueError("MAPF graph must not contain self-loops")
            if not math.isfinite(float(costs[edge_id])) or abs(float(costs[edge_id]) - 1.0) > 1e-12:
                raise ValueError("MAPF requires unit-weight edges")
            pair = (source, target)
            if pair in edge_pairs:
                raise ValueError("MAPF graph must not contain duplicate edges")
            edge_pairs.add(pair)
    if any((target, source) not in edge_pairs for source, target in edge_pairs):
        raise ValueError("MAPF requires an undirected graph")

    start_ids: list[int] = []
    goal_ids: list[int] = []
    for start, goal in zip(starts, goals, strict=True):
        start_id = graph._node_id(start)
        goal_id = graph._node_id(goal)
        if start_id is None or goal_id is None:
            raise ValueError("all multi-agent starts and goals must be graph nodes")
        start_ids.append(start_id)
        goal_ids.append(goal_id)
    return (
        graph,
        view,
        np.ascontiguousarray(start_ids, dtype=np.uint64),
        np.ascontiguousarray(goal_ids, dtype=np.uint64),
        len(starts),
    )


def _mapf_limits(params: Mapping[str, object] | None) -> tuple[int | None, float]:
    max_expansions = _max_expansions(params)
    runtime_value = 0.0 if params is None else params.get("max_runtime_ms", 0.0)
    if isinstance(runtime_value, bool) or not isinstance(runtime_value, (int, float)):
        raise TypeError("max_runtime_ms must be a finite non-negative number")
    max_runtime_ms = float(runtime_value)
    if not math.isfinite(max_runtime_ms) or max_runtime_ms < 0.0:
        raise ValueError("max_runtime_ms must be finite and non-negative")
    return max_expansions, max_runtime_ms


def _validate_paths(
    graph: NativeGraph[N], starts: Sequence[N], goals: Sequence[N], paths: tuple[tuple[N, ...], ...]
) -> tuple[int, int]:
    if len(paths) != len(starts):
        raise RuntimeError("native MAPF result returned the wrong number of paths")
    native_adjacency: list[set[int]] = [set() for _ in range(graph.node_count)]
    view = GraphCsrView()
    library = load_native_library()
    if library.pp_graph_export_csr_view(graph._native_handle, ctypes.byref(view)) != 0:
        raise RuntimeError("could not export MAPF graph for path validation")
    for source in range(int(view.node_count)):
        for edge_id in range(int(view.offsets[source]), int(view.offsets[source + 1])):
            native_adjacency[source].add(int(view.neighbor_ids[edge_id]))
    id_paths: list[list[int]] = []
    for agent, path in enumerate(paths):
        if not path or path[0] != starts[agent] or path[-1] != goals[agent]:
            raise RuntimeError("native MAPF result returned an invalid endpoint")
        ids: list[int] = []
        for state in path:
            node_id = graph._node_id(state)
            if node_id is None:
                raise RuntimeError("native MAPF result contains an unknown graph node")
            ids.append(node_id)
        if any(
            target != source and target not in native_adjacency[source]
            for source, target in zip(ids, ids[1:], strict=False)
        ):
            raise RuntimeError("native MAPF result contains an invalid transition")
        id_paths.append(ids)
    max_cost = max((len(path) - 1 for path in id_paths), default=0)
    for time_index in range(max_cost + 1):
        positions = [path[min(time_index, len(path) - 1)] for path in id_paths]
        if len(set(positions)) != len(positions):
            raise RuntimeError("native MAPF result contains a vertex conflict")
        if time_index > 0:
            previous = [path[min(time_index - 1, len(path) - 1)] for path in id_paths]
            for first in range(len(paths)):
                for second in range(first + 1, len(paths)):
                    if (
                        previous[first] != positions[first]
                        and previous[first] == positions[second]
                        and previous[second] == positions[first]
                    ):
                        raise RuntimeError("native MAPF result contains an edge-swap conflict")
    return sum(len(path) - 1 for path in id_paths), max_cost


def _run_eecbs(
    problem: MultiAgentProblem[N],
    *,
    params: Mapping[str, object] | None,
    rng: RNG | None,
    trace: TraceOptions | None,
) -> MultiAgentPlanResult[N]:
    del rng
    total_started = time.perf_counter()
    if trace is not None and type(trace) is not TraceOptions:
        raise TypeError("trace must be TraceOptions or None")
    merged_params = dict(problem.params or {})
    if params is not None:
        merged_params.update(params)
    raw_weight = merged_params.get("w", 1.1)
    if isinstance(raw_weight, bool) or not isinstance(raw_weight, (int, float)):
        raise TypeError("w must be a finite number")
    weight = float(raw_weight)
    if not math.isfinite(weight) or weight < 1.0:
        raise ValueError("w must be finite and at least 1")
    max_expansions, max_runtime_ms = _mapf_limits(merged_params)

    graph_started = time.perf_counter()
    native_graph, view, starts, goals, agent_count = _prepare_problem(problem, merged_params)
    graph_init_s = time.perf_counter() - graph_started
    library = load_search_trace_library() if trace is not None else load_native_library()
    native_result = MapfResult()
    native_trace = TraceResult() if trace is not None else None
    native_started = time.perf_counter()
    function = getattr(
        library,
        "pp_eecbs_plan_traced" if native_trace is not None else "pp_eecbs_plan",
    )
    arguments: list[Any] = [
        view.node_count,
        view.edge_count,
        view.offsets,
        view.neighbor_ids,
        starts.ctypes.data_as(ctypes.POINTER(ctypes.c_uint64)),
        goals.ctypes.data_as(ctypes.POINTER(ctypes.c_uint64)),
        agent_count,
        weight,
        int(max_expansions is not None),
        max_expansions or 0,
        max_runtime_ms,
    ]
    if native_trace is not None:
        assert trace is not None
        arguments.append(trace.max_bytes)
    arguments.append(ctypes.byref(native_result))
    if native_trace is not None:
        arguments.append(ctypes.byref(native_trace))
    status = function(*arguments)
    native_search_s = time.perf_counter() - native_started
    try:
        if status != 0:
            detail = "unknown native EECBS error"
            if native_result.error_message is not None:
                detail = native_result.error_message.decode("utf-8", errors="replace")
            raise RuntimeError(detail)
        stop_reason = _STOP_REASONS.get(native_result.stop_reason)
        if stop_reason is None:
            raise RuntimeError("native EECBS returned an unknown stop reason")
        paths: tuple[tuple[N, ...], ...] = ()
        if native_result.success:
            if native_result.agent_count != agent_count or native_result.path_offsets is None:
                raise RuntimeError("native EECBS result has invalid path offsets")
            offsets = np.ctypeslib.as_array(
                native_result.path_offsets, shape=(agent_count + 1,)
            ).copy()
            node_ids = np.ctypeslib.as_array(
                native_result.path_nodes, shape=(native_result.path_node_count,)
            ).copy()
            paths = tuple(
                tuple(
                    native_graph.node_labels[int(node_id)]
                    for node_id in node_ids[offsets[i] : offsets[i + 1]]
                )
                for i in range(agent_count)
            )
            sum_costs, makespan = _validate_paths(
                native_graph, tuple(problem.starts), tuple(problem.goals), paths
            )
            if sum_costs != native_result.sum_of_costs or makespan != native_result.makespan:
                raise RuntimeError("native EECBS objective does not match returned paths")
        planner_trace = None
        trace_graph_bytes = 0
        if native_trace is not None:
            trace_graph_bytes = int(
                (int(view.node_count) + 1) * 8
                + int(view.edge_count) * 16
                + starts.nbytes
                + goals.nbytes
            )
            planner_trace = copy_trace_result(
                native_trace,
                kind="discrete",
                node_labels=native_graph.node_labels,
                graph_bytes=trace_graph_bytes,
            )
        return MultiAgentPlanResult(
            success=bool(native_result.success),
            path=None,
            best_path=None,
            stop_reason=stop_reason,
            iters=int(native_result.iters),
            nodes=int(native_result.nodes),
            stats={
                "expanded": float(native_result.iters),
                "high_level_expanded": float(native_result.high_level_expanded),
                "low_level_expanded": float(native_result.low_level_expanded),
                "graph_init_s": graph_init_s,
                "native_search_s": native_search_s,
                "planner_total_s": time.perf_counter() - total_started,
                "suboptimality_weight": weight,
                **(
                    {"trace_graph_bytes": float(trace_graph_bytes)}
                    if planner_trace is not None
                    else {}
                ),
            },
            trace=planner_trace,
            paths=paths,
            sum_of_costs=int(native_result.sum_of_costs) if paths else 0,
            makespan=int(native_result.makespan) if paths else 0,
        )
    finally:
        library.pp_mapf_free_result(ctypes.byref(native_result))
        if native_trace is not None:
            library.pp_search_trace_free_result(ctypes.byref(native_trace))


def _run_lacam_star(
    problem: MultiAgentProblem[N],
    *,
    params: Mapping[str, object] | None,
    rng: RNG | None,
    trace: TraceOptions | None,
) -> MultiAgentPlanResult[N]:
    total_started = time.perf_counter()
    if trace is not None and type(trace) is not TraceOptions:
        raise TypeError("trace must be TraceOptions or None")
    merged_params = dict(problem.params or {})
    if params is not None:
        merged_params.update(params)
    max_expansions, max_runtime_ms = _mapf_limits(merged_params)
    random_generator = rng if rng is not None else np.random.default_rng(0)
    seed = int(random_generator.integers(0, np.iinfo(np.uint64).max, dtype=np.uint64))

    graph_started = time.perf_counter()
    native_graph, view, starts, goals, agent_count = _prepare_problem(problem, merged_params)
    graph_init_s = time.perf_counter() - graph_started
    library = load_search_trace_library() if trace is not None else load_native_library()
    native_result = MapfResult()
    native_trace = TraceResult() if trace is not None else None
    function = getattr(
        library,
        "pp_lacam_star_plan_traced" if native_trace is not None else "pp_lacam_star_plan",
    )
    arguments: list[Any] = [
        view.node_count,
        view.edge_count,
        view.offsets,
        view.neighbor_ids,
        starts.ctypes.data_as(ctypes.POINTER(ctypes.c_uint64)),
        goals.ctypes.data_as(ctypes.POINTER(ctypes.c_uint64)),
        agent_count,
        seed,
        int(max_expansions is not None),
        max_expansions or 0,
        max_runtime_ms,
    ]
    if native_trace is not None:
        assert trace is not None
        arguments.append(trace.max_bytes)
    arguments.append(ctypes.byref(native_result))
    if native_trace is not None:
        arguments.append(ctypes.byref(native_trace))
    native_started = time.perf_counter()
    status = function(*arguments)
    native_search_s = time.perf_counter() - native_started
    try:
        if status != 0:
            detail = "unknown native LaCAM* error"
            if native_result.error_message is not None:
                detail = native_result.error_message.decode("utf-8", errors="replace")
            raise RuntimeError(detail)
        stop_reason = _STOP_REASONS.get(native_result.stop_reason)
        if stop_reason is None:
            raise RuntimeError("native LaCAM* returned an unknown stop reason")
        paths: tuple[tuple[N, ...], ...] = ()
        if native_result.success:
            if native_result.agent_count != agent_count or native_result.path_offsets is None:
                raise RuntimeError("native LaCAM* result has invalid path offsets")
            offsets = np.ctypeslib.as_array(
                native_result.path_offsets, shape=(agent_count + 1,)
            ).copy()
            node_ids = np.ctypeslib.as_array(
                native_result.path_nodes, shape=(native_result.path_node_count,)
            ).copy()
            paths = tuple(
                tuple(
                    native_graph.node_labels[int(node_id)]
                    for node_id in node_ids[offsets[i] : offsets[i + 1]]
                )
                for i in range(agent_count)
            )
            sum_costs, makespan = _validate_paths(
                native_graph, tuple(problem.starts), tuple(problem.goals), paths
            )
            if sum_costs != native_result.sum_of_costs or makespan != native_result.makespan:
                raise RuntimeError("native LaCAM* objective does not match returned paths")
        planner_trace = None
        trace_graph_bytes = 0
        if native_trace is not None:
            trace_graph_bytes = int(
                (int(view.node_count) + 1) * 8
                + int(view.edge_count) * 16
                + starts.nbytes
                + goals.nbytes
            )
            planner_trace = copy_trace_result(
                native_trace,
                kind="discrete",
                node_labels=native_graph.node_labels,
                graph_bytes=trace_graph_bytes,
            )
        return MultiAgentPlanResult(
            success=bool(native_result.success),
            path=None,
            best_path=None,
            stop_reason=stop_reason,
            iters=int(native_result.iters),
            nodes=int(native_result.nodes),
            stats={
                "expanded": float(native_result.iters),
                "high_level_expanded": float(native_result.high_level_expanded),
                "low_level_expanded": float(native_result.low_level_expanded),
                "graph_init_s": graph_init_s,
                "native_search_s": native_search_s,
                "planner_total_s": time.perf_counter() - total_started,
                **(
                    {"trace_graph_bytes": float(trace_graph_bytes)}
                    if planner_trace is not None
                    else {}
                ),
            },
            trace=planner_trace,
            paths=paths,
            sum_of_costs=int(native_result.sum_of_costs) if paths else 0,
            makespan=int(native_result.makespan) if paths else 0,
        )
    finally:
        library.pp_mapf_free_result(ctypes.byref(native_result))
        if native_trace is not None:
            library.pp_search_trace_free_result(ctypes.byref(native_trace))


__all__ = ["_run_eecbs", "_run_lacam_star", "_prepare_problem", "_validate_paths"]
