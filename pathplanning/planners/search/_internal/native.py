"""ctypes adapter for the native discrete-search engine."""

from __future__ import annotations

import ctypes
from importlib.machinery import EXTENSION_SUFFIXES
import math
from pathlib import Path
import time
from typing import Any, Generic, TypeVar, cast

import numpy as np

from pathplanning.core.contracts import DiscreteProblem, HeuristicDiscreteGraph
from pathplanning.core.results import PlanResult, StopReason

N = TypeVar("N")


class NativeSearchUnavailable(RuntimeError):
    """Raised when the native search shared library is not built yet."""


class NativeSearchError(RuntimeError):
    """Raised when the native engine or graph callback reports an error."""


_GoalCallback = ctypes.CFUNCTYPE(
    ctypes.c_int,
    ctypes.c_void_p,
    ctypes.c_uint64,
    ctypes.POINTER(ctypes.c_int),
)
_HeuristicCallback = ctypes.CFUNCTYPE(
    ctypes.c_int,
    ctypes.c_void_p,
    ctypes.c_uint64,
    ctypes.POINTER(ctypes.c_double),
)
_NeighborsCallback = ctypes.CFUNCTYPE(
    ctypes.c_int,
    ctypes.c_void_p,
    ctypes.c_uint64,
    ctypes.POINTER(ctypes.POINTER(ctypes.c_uint64)),
    ctypes.POINTER(ctypes.POINTER(ctypes.c_double)),
    ctypes.POINTER(ctypes.c_size_t),
)


class _GraphCallbacks(ctypes.Structure):
    _fields_ = [
        ("user_data", ctypes.c_void_p),
        ("is_goal", _GoalCallback),
        ("heuristic", _HeuristicCallback),
        ("neighbors", _NeighborsCallback),
    ]


class _SearchOptions(ctypes.Structure):
    _fields_ = [
        ("algorithm", ctypes.c_int),
        ("has_max_expansions", ctypes.c_int),
        ("max_expansions", ctypes.c_uint64),
        ("heuristic_weight", ctypes.c_double),
        ("reserve_nodes", ctypes.c_uint64),
        ("has_goal_id", ctypes.c_int),
        ("goal_id", ctypes.c_uint64),
        ("anytime_weights", ctypes.POINTER(ctypes.c_double)),
        ("anytime_weight_count", ctypes.c_size_t),
    ]


class _SearchResult(ctypes.Structure):
    _fields_ = [
        ("success", ctypes.c_int),
        ("stop_reason", ctypes.c_int),
        ("iters", ctypes.c_uint64),
        ("nodes", ctypes.c_uint64),
        ("path_cost", ctypes.c_double),
        ("path_ids", ctypes.POINTER(ctypes.c_uint64)),
        ("path_length", ctypes.c_size_t),
        ("error_message", ctypes.c_char_p),
    ]


_NATIVE_LIB: ctypes.CDLL | None = None
_ACTIVE_ADAPTERS: dict[int, "_ProblemAdapter[Any]"] = {}
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


def _native_dir() -> Path:
    return Path(__file__).resolve().parents[3] / "native"


def _load_library() -> ctypes.CDLL:
    global _NATIVE_LIB
    if _NATIVE_LIB is not None:
        return _NATIVE_LIB

    candidates = [_native_dir() / f"_search_engine{suffix}" for suffix in EXTENSION_SUFFIXES]
    for candidate in candidates:
        if not candidate.exists():
            continue
        library = ctypes.CDLL(str(candidate))
        try:
            library.pp_search_plan.argtypes = [
                ctypes.POINTER(_GraphCallbacks),
                ctypes.c_uint64,
                ctypes.POINTER(_SearchOptions),
                ctypes.POINTER(_SearchResult),
            ]
            library.pp_search_plan.restype = ctypes.c_int
        except AttributeError as exc:
            raise NativeSearchUnavailable(
                "native search engine is stale; run `make build-ext`"
            ) from exc
        library.pp_search_free_result.argtypes = [ctypes.POINTER(_SearchResult)]
        library.pp_search_free_result.restype = None
        library.pp_search_engine_version.argtypes = []
        library.pp_search_engine_version.restype = ctypes.c_char_p
        _NATIVE_LIB = library
        return library

    searched = ", ".join(str(path) for path in candidates)
    raise NativeSearchUnavailable(
        f"native search engine is not built; run `make build-ext`. Searched: {searched}"
    )


def _adapter_from_user_data(user_data: int | None) -> "_ProblemAdapter[Any]":
    if user_data is None:
        raise RuntimeError("missing native adapter user data")
    return _ACTIVE_ADAPTERS[int(user_data)]


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
    def __init__(self, problem: DiscreteProblem[N], *, use_heuristic: bool) -> None:
        self.problem = problem
        self.graph = problem.graph
        self.goal_test = problem.resolve_goal_test()
        self.use_heuristic = use_heuristic

        goal = problem.goal
        self.exact_goal: N | None = None
        self.heuristic_graph: HeuristicDiscreteGraph[N] | None
        if not hasattr(goal, "is_goal"):
            self.exact_goal = cast(N, goal)

        if (
            use_heuristic
            and self.exact_goal is not None
            and isinstance(self.graph, HeuristicDiscreteGraph)
        ):
            self.heuristic_graph = self.graph
        else:
            self.heuristic_graph = None

        self._node_to_id: dict[N, int] = {}
        self._id_to_node: list[N] = []
        self._last_neighbor_ids: ctypes.Array[ctypes.c_uint64] | None = None
        self._last_edge_costs: ctypes.Array[ctypes.c_double] | None = None
        self.error: str | None = None

    def node_id(self, node: N) -> int:
        existing = self._node_to_id.get(node)
        if existing is not None:
            return existing
        node_id = len(self._id_to_node)
        self._node_to_id[node] = node_id
        self._id_to_node.append(node)
        return node_id

    def node_for_id(self, node_id: int) -> N:
        return self._id_to_node[node_id]

    def is_goal(self, node_id: int) -> bool:
        return bool(self.goal_test.is_goal(self.node_for_id(node_id)))

    def heuristic(self, node_id: int) -> float:
        heuristic_graph = self.heuristic_graph
        exact_goal = self.exact_goal
        if heuristic_graph is None or exact_goal is None:
            return 0.0
        value = float(heuristic_graph.heuristic(self.node_for_id(node_id), exact_goal))
        return value if math.isfinite(value) else 0.0

    def neighbors(
        self,
        node_id: int,
    ) -> tuple[ctypes.Array[ctypes.c_uint64], ctypes.Array[ctypes.c_double]]:
        node = self.node_for_id(node_id)
        neighbor_ids: list[int] = []
        edge_costs: list[float] = []

        neighbor_edges = getattr(self.graph, "neighbor_edges", None)
        if callable(neighbor_edges):
            for neighbor, edge_cost in neighbor_edges(node):
                neighbor_ids.append(self.node_id(neighbor))
                edge_costs.append(float(edge_cost))
        else:
            for neighbor in self.graph.neighbors(node):
                neighbor_ids.append(self.node_id(neighbor))
                edge_costs.append(float(self.graph.edge_cost(node, neighbor)))

        ids_array = (ctypes.c_uint64 * len(neighbor_ids))(*neighbor_ids)
        costs_array = (ctypes.c_double * len(edge_costs))(*edge_costs)
        self._last_neighbor_ids = ids_array
        self._last_edge_costs = costs_array
        return ids_array, costs_array

    def path_nodes(self, path_ids: ctypes.POINTER(ctypes.c_uint64), length: int) -> list[N]:
        return [self.node_for_id(int(path_ids[index])) for index in range(length)]

    def reserve_nodes(self) -> int:
        graph = self.graph
        dimensions = [
            getattr(graph, "x_range", None),
            getattr(graph, "y_range", None),
            getattr(graph, "z_range", None),
        ]
        product = 1
        found_dimension = False
        for value in dimensions:
            if isinstance(value, int) and value > 0:
                product *= value
                found_dimension = True
        return product if found_dimension else 0


@_GoalCallback
def _goal_callback(
    user_data: int | None,
    node_id: int,
    out_is_goal: ctypes.POINTER(ctypes.c_int),
) -> int:
    try:
        adapter = _adapter_from_user_data(user_data)
        out_is_goal[0] = 1 if adapter.is_goal(int(node_id)) else 0
        return 0
    except Exception as exc:
        if user_data is not None and int(user_data) in _ACTIVE_ADAPTERS:
            _ACTIVE_ADAPTERS[int(user_data)].error = str(exc)
        return 1


@_HeuristicCallback
def _heuristic_callback(
    user_data: int | None,
    node_id: int,
    out_value: ctypes.POINTER(ctypes.c_double),
) -> int:
    try:
        adapter = _adapter_from_user_data(user_data)
        out_value[0] = adapter.heuristic(int(node_id))
        return 0
    except Exception as exc:
        if user_data is not None and int(user_data) in _ACTIVE_ADAPTERS:
            _ACTIVE_ADAPTERS[int(user_data)].error = str(exc)
        return 1


@_NeighborsCallback
def _neighbors_callback(
    user_data: int | None,
    node_id: int,
    out_neighbor_ids: ctypes.POINTER(ctypes.POINTER(ctypes.c_uint64)),
    out_edge_costs: ctypes.POINTER(ctypes.POINTER(ctypes.c_double)),
    out_count: ctypes.POINTER(ctypes.c_size_t),
) -> int:
    try:
        adapter = _adapter_from_user_data(user_data)
        ids_array, costs_array = adapter.neighbors(int(node_id))
        out_neighbor_ids[0] = ctypes.cast(ids_array, ctypes.POINTER(ctypes.c_uint64))
        out_edge_costs[0] = ctypes.cast(costs_array, ctypes.POINTER(ctypes.c_double))
        out_count[0] = len(ids_array)
        return 0
    except Exception as exc:
        if user_data is not None and int(user_data) in _ACTIVE_ADAPTERS:
            _ACTIVE_ADAPTERS[int(user_data)].error = str(exc)
        return 1


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
    """Run one native graph-search kernel and adapt the result to Python."""
    library = _load_library()
    adapter = _ProblemAdapter(problem, use_heuristic=use_heuristic)
    start_id = adapter.node_id(problem.start)
    goal_id = 0
    has_goal_id = adapter.exact_goal is not None
    if adapter.exact_goal is not None:
        goal_id = adapter.node_id(adapter.exact_goal)
    elif require_exact_goal:
        raise NativeSearchUnavailable("native bidirectional search requires an exact goal")

    adapter_key = id(adapter)
    _ACTIVE_ADAPTERS[adapter_key] = adapter

    graph_callbacks = _GraphCallbacks(
        ctypes.c_void_p(adapter_key),
        _goal_callback,
        _heuristic_callback,
        _neighbors_callback,
    )
    weights_array: ctypes.Array[ctypes.c_double] | None = None
    weights_pointer = ctypes.POINTER(ctypes.c_double)()
    if anytime_weights:
        weights_array = (ctypes.c_double * len(anytime_weights))(*anytime_weights)
        weights_pointer = ctypes.cast(weights_array, ctypes.POINTER(ctypes.c_double))

    options = _SearchOptions(
        int(algorithm),
        1 if max_expansions is not None else 0,
        0 if max_expansions is None else int(max_expansions),
        float(heuristic_weight if use_heuristic else 0.0),
        adapter.reserve_nodes(),
        1 if has_goal_id else 0,
        int(goal_id),
        weights_pointer,
        len(anytime_weights),
    )
    result = _SearchResult()
    start_time = time.perf_counter()

    try:
        _ = weights_array
        status = int(
            library.pp_search_plan(
                ctypes.byref(graph_callbacks),
                ctypes.c_uint64(start_id),
                ctypes.byref(options),
                ctypes.byref(result),
            )
        )
        elapsed = time.perf_counter() - start_time
        if status != 0 or result.stop_reason == 3:
            native_error = (
                result.error_message.decode("utf-8", errors="replace")
                if result.error_message
                else None
            )
            raise NativeSearchError(adapter.error or native_error or "native search failed")

        stop_reason = _STOP_REASONS.get(int(result.stop_reason), StopReason.NO_PROGRESS)
        path = None
        if result.success and result.path_ids:
            path_nodes = adapter.path_nodes(result.path_ids, int(result.path_length))
            path = _path_to_matrix(path_nodes)

        stats: dict[str, float] = {
            "elapsed_s": elapsed,
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
        _ACTIVE_ADAPTERS.pop(adapter_key, None)


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
