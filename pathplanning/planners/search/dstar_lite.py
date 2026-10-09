"""Stateful native D* Lite search for discrete graphs."""

from __future__ import annotations

from collections.abc import Iterable, Mapping
import ctypes
import math
import time
from typing import Generic, TypeVar, cast
import weakref

import numpy as np

from pathplanning.core.contracts import DiscreteGraph, DiscreteProblem
from pathplanning.core.results import PlanResult, StopReason
from pathplanning.core.trace import TraceOptions
from pathplanning.core.types import RNG
from pathplanning.native import NativeGraph
from pathplanning.native._ffi import (
    DStarEdgeUpdate,
    GraphCsrView,
    SearchResult,
    TraceResult,
    copy_trace_result,
    load_native_library,
)
from pathplanning.planners.search._grid_utils import _max_expansions, _native_valid_nodes
from pathplanning.planners.search._internal.native import NativeSearchError, _ProblemAdapter
from pathplanning.spaces.grid2d import Grid2DSearchSpace

N = TypeVar("N")
_DEFAULT_MAX_MATERIALIZED_NODES = 1_000_000


def _grid_native_graph(graph: Grid2DSearchSpace, max_nodes: int) -> NativeGraph[tuple[int, int]]:
    node_count = graph.x_range * graph.y_range
    if node_count > max_nodes:
        raise ValueError("grid exceeds max_materialized_nodes")
    if (
        type(graph) is Grid2DSearchSpace
        and graph._is_blocked_callback is None
        and getattr(graph.edge_cost, "__func__", None) is Grid2DSearchSpace.edge_cost
    ):
        return _grid_native_graph_vectorized(graph)

    offsets = [0]
    neighbor_ids: list[int] = []
    edge_costs: list[float] = []
    labels = [(x, y) for y in range(graph.y_range) for x in range(graph.x_range)]
    for y in range(graph.y_range):
        for x in range(graph.x_range):
            for dx, dy in graph.motions:
                next_x, next_y = x + dx, y + dy
                if 0 <= next_x < graph.x_range and 0 <= next_y < graph.y_range:
                    neighbor_ids.append(next_y * graph.x_range + next_x)
                    edge_costs.append(float(graph.edge_cost((x, y), (next_x, next_y))))
            offsets.append(len(neighbor_ids))
    return NativeGraph.from_csr(
        offsets,
        neighbor_ids,
        edge_costs,
        node_labels=labels,
        heuristic=graph.heuristic,
    )


def _grid_native_graph_vectorized(
    graph: Grid2DSearchSpace,
) -> NativeGraph[tuple[int, int]]:
    """Materialize the built-in grid graph using NumPy rather than Python edge loops."""
    width, height = graph.x_range, graph.y_range
    node_count = width * height
    node_ids = np.arange(node_count, dtype=np.int64)
    x_coords = node_ids % width
    y_coords = node_ids // width
    valid_nodes = _native_valid_nodes(graph).astype(bool, copy=False)

    edge_masks = []
    degrees = np.zeros(node_count, dtype=np.uint64)
    for dx, dy in graph.motions:
        target_x = x_coords + dx
        target_y = y_coords + dy
        in_bounds = (target_x >= 0) & (target_x < width) & (target_y >= 0) & (target_y < height)
        edge_masks.append((dx, dy, in_bounds))
        degrees += in_bounds

    offsets = np.empty(node_count + 1, dtype=np.uint64)
    offsets[0] = 0
    np.cumsum(degrees, out=offsets[1:])
    edge_count = int(offsets[-1])
    neighbor_ids = np.empty(edge_count, dtype=np.uint64)
    edge_costs = np.empty(edge_count, dtype=np.float64)
    cursors = offsets[:-1].copy()

    for dx, dy, in_bounds in edge_masks:
        sources = np.flatnonzero(in_bounds)
        target_x = x_coords[sources] + dx
        target_y = y_coords[sources] + dy
        targets = target_y * width + target_x
        positions = cursors[sources]
        neighbor_ids[positions] = targets

        costs = np.full(sources.size, np.inf, dtype=np.float64)
        if max(abs(dx), abs(dy)) <= 1 and (dx != 0 or dy != 0):
            edge_is_valid = valid_nodes[sources] & valid_nodes[targets]
            costs[edge_is_valid] = np.hypot(float(dx), float(dy))
        edge_costs[positions] = costs
        cursors[sources] += 1

    labels = [(x, y) for y in range(height) for x in range(width)]
    return NativeGraph.from_csr(
        offsets,
        neighbor_ids,
        edge_costs,
        node_labels=labels,
        heuristic=graph.heuristic,
    )


class DStarLitePlanner(Generic[N]):
    """Native D* Lite session; update edge costs and move the start between plans."""

    def __init__(
        self,
        graph: DiscreteGraph[N] | NativeGraph[N],
        start: N,
        goal: N,
        *,
        use_heuristic: bool = True,
        max_materialized_nodes: int = _DEFAULT_MAX_MATERIALIZED_NODES,
    ) -> None:
        max_materialized_nodes = _validate_node_limit(max_materialized_nodes)
        if hasattr(goal, "is_goal"):
            raise TypeError("D* Lite requires an exact goal node")
        if isinstance(graph, Grid2DSearchSpace) and (
            not graph.is_valid_node(cast(tuple[int, int], start))
            or not graph.is_valid_node(cast(tuple[int, int], goal))
        ):
            raise ValueError("D* Lite start and goal must be valid grid cells")
        graph_started = time.perf_counter()
        self.graph = graph
        self.start = start
        self.goal = goal
        self._use_heuristic = use_heuristic
        self._adapter: _ProblemAdapter[N] | None = None
        if isinstance(graph, Grid2DSearchSpace):
            native_graph = cast(NativeGraph[N], _grid_native_graph(graph, max_materialized_nodes))
        else:
            problem = DiscreteProblem(
                graph,
                start,
                goal,
                params={"max_materialized_nodes": max_materialized_nodes},
            )
            self._adapter = _ProblemAdapter(problem, use_heuristic=use_heuristic)
            native_graph = self._adapter.native_graph
        self._native_graph = native_graph
        self._labels = native_graph.node_labels
        self._start_id = self._node_id(start)
        self._goal_id = self._node_id(goal)
        self._heuristic_values = self._build_heuristic_values(self._start_id)
        self._heuristic_array = np.ascontiguousarray(self._heuristic_values, dtype=np.float64)

        self._library = load_native_library()
        view = GraphCsrView()
        status = self._library.pp_graph_export_csr_view(
            native_graph._native_handle,
            ctypes.byref(view),
        )
        if status != 0:
            raise NativeSearchError("could not export graph adjacency for D* Lite")
        handle = ctypes.c_void_p()
        error_buffer = ctypes.create_string_buffer(512)
        heuristic_pointer = self._heuristic_array.ctypes.data_as(ctypes.POINTER(ctypes.c_double))
        status = self._library.pp_dstar_lite_create(
            view.node_count,
            view.edge_count,
            view.offsets,
            view.neighbor_ids,
            view.edge_costs,
            self._start_id,
            self._goal_id,
            heuristic_pointer,
            ctypes.byref(handle),
            error_buffer,
            len(error_buffer),
        )
        if status != 0 or handle.value is None:
            message = error_buffer.value.decode("utf-8", errors="replace")
            raise NativeSearchError(message or "native D* Lite initialization failed")
        self._handle = handle
        self._finalizer = weakref.finalize(self, self._library.pp_dstar_lite_free, handle)
        self._graph_init_s = time.perf_counter() - graph_started
        self._graph_bytes = int((view.node_count + 1) * 8 + view.edge_count * 16)
        self._plan_calls = 0

    def _node_id(self, node: N) -> int:
        node_id = self._native_graph._node_id(node)
        if node_id is None:
            raise ValueError(f"node is not present in the D* Lite graph: {node!r}")
        return node_id

    def _ensure_open(self) -> None:
        if not self._finalizer.alive:
            raise RuntimeError("D* Lite planner is closed")

    def _heuristic(self, source_id: int, target_id: int) -> float:
        if not self._use_heuristic:
            return 0.0
        graph = self._native_graph
        target = graph._node_for_id(target_id)
        if graph._heuristic_fn is not None:
            value = graph._heuristic(source_id, target)
        elif graph._heuristic_mode == "euclidean":
            source = graph._node_for_id(source_id)
            value = math.dist(cast(tuple[float, ...], source), cast(tuple[float, ...], target))
        else:
            value = 0.0
        value = float(value)
        if not math.isfinite(value) or value < 0.0:
            raise ValueError("graph heuristic must return finite non-negative values")
        return value

    def _build_heuristic_values(self, start_id: int) -> np.ndarray:
        if not self._use_heuristic:
            return np.zeros(self._native_graph.node_count, dtype=np.float64)
        graph = self.graph
        if (
            isinstance(graph, Grid2DSearchSpace)
            and getattr(graph.heuristic, "__func__", None) is Grid2DSearchSpace.heuristic
            and getattr(graph.native_heuristic_values, "__func__", None)
            is Grid2DSearchSpace.native_heuristic_values
        ):
            start = (start_id % graph.x_range, start_id // graph.x_range)
            return np.ascontiguousarray(graph.native_heuristic_values(start), dtype=np.float64)
        return np.fromiter(
            (
                self._heuristic(start_id, node_id)
                for node_id in range(self._native_graph.node_count)
            ),
            dtype=np.float64,
            count=self._native_graph.node_count,
        )

    def _call_update(self, function: str, *arguments: object) -> None:
        self._ensure_open()
        error_buffer = ctypes.create_string_buffer(512)
        status = getattr(self._library, function)(
            self._handle,
            *arguments,
            error_buffer,
            len(error_buffer),
        )
        if status != 0:
            message = error_buffer.value.decode("utf-8", errors="replace")
            raise ValueError(message or f"native D* Lite {function} failed")

    def move_start(self, start: N) -> None:
        """Move the start and keep the current search values and queue."""
        start_id = self._node_id(start)
        values = np.ascontiguousarray(self._build_heuristic_values(start_id), dtype=np.float64)
        pointer = values.ctypes.data_as(ctypes.POINTER(ctypes.c_double))
        delta = self._heuristic(self._start_id, start_id)
        self._call_update("pp_dstar_lite_move_start", start_id, pointer, delta)
        self.start = start
        self._start_id = start_id
        self._heuristic_array = values

    def update_edges(self, updates: Iterable[tuple[N, N, float | None]]) -> None:
        """Update directed edges; use ``None`` to restore their initial costs."""
        self._ensure_open()
        rows: list[DStarEdgeUpdate] = []
        seen: set[tuple[int, int]] = set()
        for source, target, raw_cost in updates:
            source_id, target_id = self._node_id(source), self._node_id(target)
            if (source_id, target_id) in seen:
                raise ValueError("edge update batch contains duplicate directed edges")
            seen.add((source_id, target_id))
            if raw_cost is None:
                rows.append(DStarEdgeUpdate(source_id, target_id, 0.0, 1))
                continue
            if isinstance(raw_cost, bool):
                raise TypeError("edge cost must be a non-negative number")
            cost = float(raw_cost)
            if math.isnan(cost) or cost < 0.0:
                raise ValueError("edge cost must be non-negative and not NaN")
            rows.append(DStarEdgeUpdate(source_id, target_id, cost, 0))
        update_array = (DStarEdgeUpdate * len(rows))(*rows) if rows else None
        pointer = (
            ctypes.cast(update_array, ctypes.POINTER(DStarEdgeUpdate))
            if update_array is not None
            else ctypes.POINTER(DStarEdgeUpdate)()
        )
        error_buffer = ctypes.create_string_buffer(512)
        status = self._library.pp_dstar_lite_update_edges(
            self._handle,
            pointer,
            len(rows),
            error_buffer,
            len(error_buffer),
        )
        if status != 0:
            message = error_buffer.value.decode("utf-8", errors="replace")
            raise ValueError(message or "native D* Lite edge update failed")

    def reset(self, start: N | None = None, goal: N | None = None) -> None:
        """Discard cached search state and reinitialize the current query."""
        next_start = self.start if start is None else start
        next_goal = self.goal if goal is None else goal
        start_id, goal_id = self._node_id(next_start), self._node_id(next_goal)
        values = np.ascontiguousarray(self._build_heuristic_values(start_id), dtype=np.float64)
        pointer = values.ctypes.data_as(ctypes.POINTER(ctypes.c_double))
        self._call_update("pp_dstar_lite_reset", start_id, goal_id, pointer)
        self.start, self.goal = next_start, next_goal
        self._start_id, self._goal_id = start_id, goal_id
        self._heuristic_array = values
        self._plan_calls = 0

    def plan(
        self,
        *,
        max_expansions: int | None = None,
        max_runtime_ms: float | None = None,
        trace: TraceOptions | None = None,
    ) -> PlanResult:
        """Repair the route to the current goal and return the best valid path."""
        self._ensure_open()
        if max_expansions is not None and (
            isinstance(max_expansions, bool)
            or type(max_expansions) is not int
            or max_expansions <= 0
        ):
            raise ValueError("max_expansions must be a positive integer")
        runtime_limit = 0.0 if max_runtime_ms is None else _validate_runtime_limit(max_runtime_ms)
        native_result = SearchResult()
        native_trace = TraceResult() if trace is not None else None
        started = time.perf_counter()
        if native_trace is None:
            status = self._library.pp_dstar_lite_plan(
                self._handle,
                int(max_expansions is not None),
                0 if max_expansions is None else max_expansions,
                runtime_limit,
                ctypes.byref(native_result),
            )
        else:
            assert trace is not None
            status = self._library.pp_dstar_lite_plan_traced(
                self._handle,
                int(max_expansions is not None),
                0 if max_expansions is None else max_expansions,
                runtime_limit,
                trace.max_bytes,
                ctypes.byref(native_result),
                ctypes.byref(native_trace),
            )
        native_search_s = time.perf_counter() - started
        try:
            if status != 0:
                message = native_result.error_message
                detail = (
                    "unknown native D* Lite error" if message is None else message.decode("utf-8")
                )
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
                raise RuntimeError("native D* Lite returned an unknown stop reason") from exc

            path = None
            if native_result.success:
                path_ids = np.ctypeslib.as_array(
                    native_result.path_ids,
                    shape=(native_result.path_length,),
                )
                labels = [self._labels[int(node_id)] for node_id in path_ids]
                try:
                    path = np.asarray(labels, dtype=float)
                except (TypeError, ValueError):
                    path = None
                if path is not None and path.ndim == 1:
                    path = path.reshape(-1, 1)
            planner_trace = None
            if native_trace is not None:
                planner_trace = copy_trace_result(
                    native_trace,
                    kind="discrete",
                    node_labels=self._labels,
                    graph_bytes=self._graph_bytes,
                )
            self._plan_calls += 1
            return PlanResult(
                success=bool(native_result.success),
                path=path,
                best_path=path,
                stop_reason=reason,
                iters=int(native_result.iters),
                nodes=int(native_result.nodes),
                stats={
                    "expanded": float(native_result.iters),
                    "graph_init_s": self._graph_init_s,
                    "native_search_s": native_search_s,
                    "path_cost": float(native_result.path_cost),
                    "plan_calls": float(self._plan_calls),
                    "planner_total_s": time.perf_counter() - started,
                    **(
                        {"trace_graph_bytes": float(self._graph_bytes)}
                        if planner_trace is not None
                        else {}
                    ),
                },
                trace=planner_trace,
            )
        finally:
            self._library.pp_search_free_result(ctypes.byref(native_result))
            if native_trace is not None:
                self._library.pp_dstar_lite_trace_free_result(ctypes.byref(native_trace))

    def close(self) -> None:
        """Release native search state; this planner cannot be used afterward."""
        self._finalizer()


def plan_dstar_lite(
    problem: DiscreteProblem[N],
    *,
    params: Mapping[str, object] | None = None,
    rng: RNG | None = None,
    trace: TraceOptions | None = None,
) -> PlanResult:
    """Run one D* Lite query; reuse :class:`DStarLitePlanner` for updates."""
    del rng
    total_started = time.perf_counter()
    if hasattr(problem.goal, "is_goal"):
        raise TypeError("D* Lite requires an exact goal node")
    max_nodes = _DEFAULT_MAX_MATERIALIZED_NODES
    if problem.params is not None and "max_materialized_nodes" in problem.params:
        max_nodes = _validate_node_limit(problem.params["max_materialized_nodes"])
    if params is not None and "max_materialized_nodes" in params:
        max_nodes = _validate_node_limit(params["max_materialized_nodes"])
    planner = DStarLitePlanner(
        problem.graph,
        problem.start,
        cast(N, problem.goal),
        max_materialized_nodes=max_nodes,
    )
    try:
        result = planner.plan(
            max_expansions=_max_expansions(params),
            max_runtime_ms=_max_runtime_ms(params),
            trace=trace,
        )
        result.stats = {
            **result.stats,
            "planner_total_s": time.perf_counter() - total_started,
        }
        return result
    finally:
        planner.close()


def _validate_node_limit(value: object) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise TypeError("max_materialized_nodes must be an integer")
    if value <= 0:
        raise ValueError("max_materialized_nodes must be positive")
    return value


def _max_runtime_ms(params: Mapping[str, object] | None) -> float:
    value = 0.0 if params is None else params.get("max_runtime_ms", 0.0)
    return _validate_runtime_limit(value)


def _validate_runtime_limit(value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError("max_runtime_ms must be a finite non-negative number")
    runtime_limit = float(value)
    if not math.isfinite(runtime_limit) or runtime_limit < 0.0:
        raise ValueError("max_runtime_ms must be finite and non-negative")
    return runtime_limit


__all__ = ["DStarLitePlanner", "plan_dstar_lite"]
