"""Native adapter for discrete search without callbacks in the search loop."""

from __future__ import annotations

import ctypes
import math
import time
from typing import Any, Generic, TypeVar, cast

import numpy as np

from pathplanning.core.contracts import (
    DiscreteProblem,
    HeuristicDiscreteGraph,
)
from pathplanning.core.results import PlanResult, StopReason
from pathplanning.native import NativeGraph
from pathplanning.native._ffi import SearchOptions, SearchResult, load_native_library

N = TypeVar("N")


class NativeSearchUnavailable(RuntimeError):
    """Raised when the native search shared library is not built yet."""


class NativeSearchError(RuntimeError):
    """Raised when the native engine or graph initialization reports an error."""


_STOP_REASONS = {
    0: StopReason.SUCCESS,
    1: StopReason.MAX_ITERS,
    2: StopReason.NO_PROGRESS,
}
_ALGORITHM_BFS = 1
_ALGORITHM_DFS = 2
_ALGORITHM_GREEDY_BEST_FIRST = 3
_ALGORITHM_ASTAR = 4
_ALGORITHM_DIJKSTRA = 5
_ALGORITHM_WEIGHTED_ASTAR = 6
_ALGORITHM_BIDIRECTIONAL_ASTAR = 7
_ALGORITHM_ANYTIME_ASTAR = 8
_DEFAULT_MAX_MATERIALIZED_NODES = 1_000_000


def _load_library() -> ctypes.CDLL:
    try:
        return load_native_library()
    except RuntimeError as exc:
        raise NativeSearchUnavailable(str(exc)) from exc


def _path_to_matrix(path_nodes: list[Any]) -> np.ndarray | None:
    if not path_nodes:
        return None
    try:
        path_array = np.asarray(path_nodes, dtype=float)
    except Exception:
        return None
    if path_array.ndim == 1:
        path_array = path_array.reshape(-1, 1)
    return path_array.astype(float, copy=False)


class _ProblemAdapter(Generic[N]):
    """Prepare one immutable native graph and per-node query data."""

    def __init__(
        self,
        problem: DiscreteProblem[N],
        *,
        use_heuristic: bool,
    ) -> None:
        self.problem = problem
        self.source_graph = problem.graph
        self.goal_test = problem.resolve_goal_test()
        self.use_heuristic = use_heuristic

        self._has_exact_goal = not hasattr(problem.goal, "is_goal")
        self.exact_goal: N | None = cast(N, problem.goal) if self._has_exact_goal else None
        self.native_graph = self._prepare_graph(problem)
        self.id_to_node = self.native_graph.node_labels

        start_id = self.native_graph._node_id(problem.start)
        if start_id is None:
            raise NativeSearchError("start node is not present in the native graph")
        self.start_id = start_id

        if self._has_exact_goal:
            exact_goal = cast(N, problem.goal)
            resolved_goal_id = self.native_graph._node_id(exact_goal)
            self.goal_id = self.start_id if resolved_goal_id is None else resolved_goal_id
            self.has_goal_id = True
        else:
            self.goal_id = 0
            self.has_goal_id = False

        self.goal_flags = self._build_goal_flags()
        self.heuristic_values = self._build_heuristic_values()

    def _prepare_graph(self, problem: DiscreteProblem[N]) -> NativeGraph[N]:
        if isinstance(problem.graph, NativeGraph):
            return problem.graph

        limit = _DEFAULT_MAX_MATERIALIZED_NODES
        if problem.params is not None and "max_materialized_nodes" in problem.params:
            configured_limit = problem.params["max_materialized_nodes"]
            if isinstance(configured_limit, bool) or not isinstance(configured_limit, int):
                raise NativeSearchError("max_materialized_nodes must be a positive integer")
            if configured_limit <= 0:
                raise NativeSearchError("max_materialized_nodes must be a positive integer")
            limit = configured_limit

        native_factory = getattr(problem.graph, "to_native_graph", None)
        if callable(native_factory):
            dimensions = [
                getattr(problem.graph, "x_range", None),
                getattr(problem.graph, "y_range", None),
                getattr(problem.graph, "z_range", None),
            ]
            node_count = 1
            for dimension in dimensions:
                if isinstance(dimension, int) and dimension > 0:
                    node_count *= dimension
            if node_count > limit:
                raise NativeSearchError(
                    "grid exceeds max_materialized_nodes; raise the limit or use NativeGraph"
                )
            try:
                return cast(NativeGraph[N], native_factory())
            except (TypeError, ValueError, RuntimeError) as exc:
                raise NativeSearchError(f"failed to initialize native graph: {exc}") from exc

        labels: list[N] = []
        node_to_id: dict[N, int] = {}
        rows: list[list[tuple[int, float]]] = []

        def ensure_node(node: N) -> int:
            try:
                existing_id = node_to_id.get(node)
            except TypeError as exc:
                raise NativeSearchError("discrete graph nodes must be hashable") from exc
            if existing_id is not None:
                return existing_id
            if len(labels) >= limit:
                raise NativeSearchError(
                    "reachable graph exceeds max_materialized_nodes; "
                    "construct a NativeGraph from CSR arrays or raise the limit"
                )
            node_id = len(labels)
            node_to_id[node] = node_id
            labels.append(node)
            rows.append([])
            return node_id

        start_id = ensure_node(problem.start)
        queue = [start_id]
        queued = {start_id}
        if self._has_exact_goal:
            ensure_node(cast(N, problem.goal))

        cursor = 0
        neighbor_edges = getattr(problem.graph, "neighbor_edges", None)
        while cursor < len(queue):
            node_id = queue[cursor]
            node = labels[node_id]
            row: list[tuple[int, float]] = []
            if callable(neighbor_edges):
                edges = neighbor_edges(node)
                for neighbor, edge_cost in edges:
                    neighbor_id = ensure_node(neighbor)
                    row.append((neighbor_id, float(edge_cost)))
                    if neighbor_id not in queued:
                        queue.append(neighbor_id)
                        queued.add(neighbor_id)
            else:
                for neighbor in problem.graph.neighbors(node):
                    neighbor_id = ensure_node(neighbor)
                    row.append((neighbor_id, float(problem.graph.edge_cost(node, neighbor))))
                    if neighbor_id not in queued:
                        queue.append(neighbor_id)
                        queued.add(neighbor_id)
            rows[node_id] = row
            cursor += 1

        offsets = [0]
        neighbor_ids: list[int] = []
        edge_costs: list[float] = []
        for row in rows:
            for neighbor_id, edge_cost in row:
                neighbor_ids.append(neighbor_id)
                edge_costs.append(edge_cost)
            offsets.append(len(neighbor_ids))

        heuristic = None
        if self.use_heuristic and self._has_exact_goal:
            candidate = getattr(problem.graph, "heuristic", None)
            if callable(candidate):
                heuristic = candidate

        try:
            return NativeGraph.from_csr(
                offsets,
                neighbor_ids,
                edge_costs,
                node_labels=labels,
                heuristic=heuristic,
            )
        except (TypeError, ValueError, RuntimeError) as exc:
            raise NativeSearchError(f"failed to initialize native graph: {exc}") from exc

    def _build_goal_flags(self) -> np.ndarray:
        flags = np.zeros(self.native_graph.node_count, dtype=np.uint8)
        if self._has_exact_goal:
            goal_id = self.native_graph._node_id(cast(N, self.problem.goal))
            if goal_id is not None:
                flags[goal_id] = 1
            return flags

        for node_id, node in enumerate(self.id_to_node):
            flags[node_id] = 1 if self.goal_test.is_goal(node) else 0
        return flags

    def _build_heuristic_values(self) -> np.ndarray:
        values = np.zeros(self.native_graph.node_count, dtype=np.float64)
        if not self.use_heuristic or not self._has_exact_goal:
            return values

        exact_goal = cast(N, self.problem.goal)
        if isinstance(self.source_graph, NativeGraph):
            get_value = self.source_graph._heuristic
            for node_id in range(self.native_graph.node_count):
                value = get_value(node_id, exact_goal)
                values[node_id] = value if math.isfinite(value) else 0.0
            return values

        value_builder = getattr(self.source_graph, "native_heuristic_values", None)
        if callable(value_builder):
            built_values = np.asarray(value_builder(exact_goal), dtype=np.float64)
            if built_values.shape != values.shape:
                raise NativeSearchError(
                    "native_heuristic_values must return one value per native graph node"
                )
            values[:] = built_values
            values[~np.isfinite(values)] = 0.0
            return values

        if isinstance(self.source_graph, HeuristicDiscreteGraph):
            for node_id, node in enumerate(self.id_to_node):
                value = float(self.source_graph.heuristic(node, exact_goal))
                values[node_id] = value if math.isfinite(value) else 0.0
        return values

    def path_nodes(self, path_ids: ctypes.POINTER(ctypes.c_uint64), length: int) -> list[N]:
        return [self.id_to_node[int(path_ids[index])] for index in range(length)]


def run_native_search(
    problem: DiscreteProblem[N],
    *,
    algorithm: int,
    max_expansions: int | None,
    use_heuristic: bool,
    heuristic_weight: float = 1.0,
    require_exact_goal: bool = False,
    anytime_weights: tuple[float, ...] = (),
) -> PlanResult:
    """Run a native graph-search kernel after one-time graph preparation."""
    library = _load_library()
    if require_exact_goal and hasattr(problem.goal, "is_goal"):
        raise NativeSearchUnavailable("native bidirectional search requires an exact goal")

    total_start = time.perf_counter()
    adapter = _ProblemAdapter(problem, use_heuristic=use_heuristic)
    graph_init_s = time.perf_counter() - total_start

    weights_array: np.ndarray | None = None
    weights_pointer = ctypes.POINTER(ctypes.c_double)()
    if anytime_weights:
        weights_array = np.ascontiguousarray(anytime_weights, dtype=np.float64)
        weights_pointer = weights_array.ctypes.data_as(ctypes.POINTER(ctypes.c_double))

    options = SearchOptions(
        int(algorithm),
        1 if max_expansions is not None else 0,
        0 if max_expansions is None else int(max_expansions),
        float(heuristic_weight if use_heuristic else 0.0),
        0,
        1 if adapter.has_goal_id else 0,
        int(adapter.goal_id),
        weights_pointer,
        len(anytime_weights),
    )
    result = SearchResult()
    goal_pointer = adapter.goal_flags.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8))
    heuristic_pointer = adapter.heuristic_values.ctypes.data_as(ctypes.POINTER(ctypes.c_double))
    native_start = time.perf_counter()

    try:
        _ = weights_array
        status = int(
            library.pp_native_search_plan(
                adapter.native_graph._native_handle,
                goal_pointer,
                heuristic_pointer,
                ctypes.c_uint64(adapter.start_id),
                ctypes.byref(options),
                ctypes.byref(result),
            )
        )
        native_search_s = time.perf_counter() - native_start
        if status != 0 or result.stop_reason == 3:
            native_error = (
                result.error_message.decode("utf-8", errors="replace")
                if result.error_message
                else None
            )
            raise NativeSearchError(native_error or "native search failed")

        stop_reason = _STOP_REASONS.get(int(result.stop_reason), StopReason.NO_PROGRESS)
        path = None
        if result.success and result.path_ids:
            path_nodes = adapter.path_nodes(result.path_ids, int(result.path_length))
            path = _path_to_matrix(path_nodes)

        stats: dict[str, float] = {
            "elapsed_s": time.perf_counter() - total_start,
            "graph_init_s": graph_init_s,
            "native_search_s": native_search_s,
            "expanded": float(result.iters),
        }
        if result.success:
            stats["path_cost"] = float(result.path_cost)
            if path is not None:
                stats["path_length"] = float(path.shape[0])

        return PlanResult(
            success=bool(result.success),
            path=path,
            best_path=path if result.success else None,
            stop_reason=stop_reason,
            iters=int(result.iters),
            nodes=int(result.nodes),
            stats=stats,
        )
    finally:
        library.pp_search_free_result(ctypes.byref(result))


def run_native_best_first(
    problem: DiscreteProblem[N],
    *,
    max_expansions: int | None,
    use_heuristic: bool,
    heuristic_weight: float = 1.0,
) -> PlanResult:
    """Run the native best-first family kernel."""
    if not use_heuristic:
        algorithm = _ALGORITHM_DIJKSTRA
    elif heuristic_weight == 1.0:
        algorithm = _ALGORITHM_ASTAR
    else:
        algorithm = _ALGORITHM_WEIGHTED_ASTAR
    return run_native_search(
        problem,
        algorithm=algorithm,
        max_expansions=max_expansions,
        use_heuristic=use_heuristic,
        heuristic_weight=heuristic_weight,
    )


def run_native_breadth_first(
    problem: DiscreteProblem[N],
    *,
    max_expansions: int | None,
) -> PlanResult:
    """Run the native breadth-first kernel."""
    return run_native_search(
        problem,
        algorithm=_ALGORITHM_BFS,
        max_expansions=max_expansions,
        use_heuristic=False,
    )


def run_native_depth_first(
    problem: DiscreteProblem[N],
    *,
    max_expansions: int | None,
) -> PlanResult:
    """Run the native depth-first kernel."""
    return run_native_search(
        problem,
        algorithm=_ALGORITHM_DFS,
        max_expansions=max_expansions,
        use_heuristic=False,
    )


def run_native_greedy_best_first(
    problem: DiscreteProblem[N],
    *,
    max_expansions: int | None,
) -> PlanResult:
    """Run the native greedy best-first kernel."""
    return run_native_search(
        problem,
        algorithm=_ALGORITHM_GREEDY_BEST_FIRST,
        max_expansions=max_expansions,
        use_heuristic=True,
    )


def run_native_bidirectional_astar(
    problem: DiscreteProblem[N],
    *,
    max_expansions: int | None,
) -> PlanResult:
    """Run the native bidirectional search kernel."""
    return run_native_search(
        problem,
        algorithm=_ALGORITHM_BIDIRECTIONAL_ASTAR,
        max_expansions=max_expansions,
        use_heuristic=False,
        require_exact_goal=True,
    )


def run_native_anytime_astar(
    problem: DiscreteProblem[N],
    *,
    max_expansions: int | None,
    weights: tuple[float, ...],
) -> PlanResult:
    """Run the native Anytime A* kernel."""
    result = run_native_search(
        problem,
        algorithm=_ALGORITHM_ANYTIME_ASTAR,
        max_expansions=max_expansions,
        use_heuristic=True,
        anytime_weights=weights,
    )
    stats = dict(result.stats)
    stats["attempts"] = float(len(weights))
    return PlanResult(
        success=result.success,
        path=result.path,
        best_path=result.best_path,
        stop_reason=result.stop_reason,
        iters=result.iters,
        nodes=result.nodes,
        stats=stats,
    )


def native_search_available() -> bool:
    """Return whether the native search shared library can be loaded."""
    try:
        _load_library()
    except NativeSearchUnavailable:
        return False
    return True


__all__ = [
    "NativeSearchError",
    "NativeSearchUnavailable",
    "native_search_available",
    "run_native_anytime_astar",
    "run_native_best_first",
    "run_native_bidirectional_astar",
    "run_native_breadth_first",
    "run_native_depth_first",
    "run_native_greedy_best_first",
    "run_native_search",
]
