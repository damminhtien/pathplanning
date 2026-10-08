"""Private ctypes declarations for native search and sampling engines."""

from __future__ import annotations

from collections.abc import Sequence
import ctypes
from importlib.machinery import EXTENSION_SUFFIXES
from pathlib import Path
from typing import Any, Literal

import numpy as np

from pathplanning.core.trace import TRACE_EVENT_DTYPE, PlannerTrace


class SearchOptions(ctypes.Structure):
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
        ("reopen_threshold", ctypes.c_double),
        ("reopen_mode", ctypes.c_int),
        ("tie_break", ctypes.c_int),
        ("max_runtime_ms", ctypes.c_double),
    ]


class SearchResult(ctypes.Structure):
    _fields_ = [
        ("success", ctypes.c_int),
        ("stop_reason", ctypes.c_int),
        ("iters", ctypes.c_uint64),
        ("reopens", ctypes.c_uint64),
        ("nodes", ctypes.c_uint64),
        ("path_cost", ctypes.c_double),
        ("path_ids", ctypes.POINTER(ctypes.c_uint64)),
        ("path_length", ctypes.c_size_t),
        ("error_message", ctypes.c_char_p),
    ]


class SippResult(ctypes.Structure):
    _fields_ = [
        ("success", ctypes.c_int),
        ("stop_reason", ctypes.c_int),
        ("iters", ctypes.c_uint64),
        ("nodes", ctypes.c_uint64),
        ("path_cost", ctypes.c_double),
        ("arrival_time", ctypes.c_double),
        ("path_ids", ctypes.POINTER(ctypes.c_uint64)),
        ("arrival_times", ctypes.POINTER(ctypes.c_double)),
        ("path_length", ctypes.c_size_t),
        ("error_message", ctypes.c_char_p),
    ]


class MapfResult(ctypes.Structure):
    _fields_ = [
        ("success", ctypes.c_int),
        ("stop_reason", ctypes.c_int),
        ("iters", ctypes.c_uint64),
        ("nodes", ctypes.c_uint64),
        ("high_level_expanded", ctypes.c_uint64),
        ("low_level_expanded", ctypes.c_uint64),
        ("sum_of_costs", ctypes.c_uint64),
        ("makespan", ctypes.c_uint64),
        ("path_offsets", ctypes.POINTER(ctypes.c_uint64)),
        ("path_nodes", ctypes.POINTER(ctypes.c_uint64)),
        ("agent_count", ctypes.c_size_t),
        ("path_node_count", ctypes.c_size_t),
        ("error_message", ctypes.c_char_p),
    ]


class JpsMetrics(ctypes.Structure):
    _fields_ = [
        ("struct_size", ctypes.c_uint64),
        ("motion_checks", ctypes.c_uint64),
        ("jump_points_expanded", ctypes.c_uint64),
    ]

    def __init__(self) -> None:
        super().__init__()
        self.struct_size = ctypes.sizeof(type(self))


class JpswMetrics(ctypes.Structure):
    _fields_ = [
        ("struct_size", ctypes.c_uint64),
        ("motion_checks", ctypes.c_uint64),
        ("neighborhood_checks", ctypes.c_uint64),
        ("prospective_prunes", ctypes.c_uint64),
        ("jump_points_expanded", ctypes.c_uint64),
    ]

    def __init__(self) -> None:
        super().__init__()
        self.struct_size = ctypes.sizeof(type(self))


class ThetaMetrics(ctypes.Structure):
    _fields_ = [
        ("struct_size", ctypes.c_uint64),
        ("line_of_sight_checks", ctypes.c_uint64),
        ("cells_checked", ctypes.c_uint64),
        ("expanded_nodes", ctypes.c_uint64),
    ]

    def __init__(self) -> None:
        super().__init__()
        self.struct_size = ctypes.sizeof(type(self))


class DStarEdgeUpdate(ctypes.Structure):
    _fields_ = [
        ("source_id", ctypes.c_uint64),
        ("target_id", ctypes.c_uint64),
        ("cost", ctypes.c_double),
        ("restore_base_cost", ctypes.c_int),
    ]


class TraceEvent(ctypes.Structure):
    _fields_ = [
        ("node", ctypes.c_uint64),
        ("parent", ctypes.c_uint64),
        ("value", ctypes.c_double),
        ("kind", ctypes.c_uint32),
        ("side", ctypes.c_uint32),
    ]


class TraceResult(ctypes.Structure):
    _fields_ = [
        ("events", ctypes.POINTER(TraceEvent)),
        ("event_count", ctypes.c_size_t),
        ("points", ctypes.POINTER(ctypes.c_double)),
        ("point_count", ctypes.c_size_t),
        ("dimension", ctypes.c_size_t),
        ("truncated", ctypes.c_int),
    ]


class GraphCsrView(ctypes.Structure):
    _fields_ = [
        ("node_count", ctypes.c_uint64),
        ("edge_count", ctypes.c_uint64),
        ("offsets", ctypes.POINTER(ctypes.c_uint64)),
        ("neighbor_ids", ctypes.POINTER(ctypes.c_uint64)),
        ("edge_costs", ctypes.POINTER(ctypes.c_double)),
        ("grid_width", ctypes.c_uint64),
        ("grid_height", ctypes.c_uint64),
        ("grid_depth", ctypes.c_uint64),
        ("euclidean_grid_heuristic", ctypes.c_int),
    ]


class GraphStorageInfo(ctypes.Structure):
    _fields_ = [
        ("struct_size", ctypes.c_uint64),
        ("graph_object_bytes", ctypes.c_uint64),
        ("base_csr_capacity_bytes", ctypes.c_uint64),
        ("reverse_csr_capacity_bytes", ctypes.c_uint64),
        ("node_count", ctypes.c_uint64),
        ("edge_count", ctypes.c_uint64),
    ]

    def __init__(self) -> None:
        super().__init__()
        self.struct_size = ctypes.sizeof(type(self))


class SearchMetrics(ctypes.Structure):
    _fields_ = [("struct_size", ctypes.c_uint64), ("capability_bits", ctypes.c_uint64)] + [
        (field, ctypes.c_uint64)
        for field in (
            "expanded",
            "reopens",
            "expanded_forward",
            "expanded_backward",
            "discovered_first",
            "discovered_forward",
            "discovered_backward",
            "edges_examined",
            "relaxation_attempts",
            "relaxation_successes_first",
            "relaxation_successes_improved",
            "closed_neighbor_skips",
            "nonimproving_skips",
            "greedy_known_skips",
            "frontier_pushes",
            "frontier_pops",
            "stale_pops",
            "frontier_peak_entries",
            "frontier_peak_forward",
            "frontier_peak_backward",
            "goal_tests",
            "heuristic_lookups",
            "heuristic_computations",
            "validation_heuristic_lookups",
            "validation_heuristic_computations",
            "validation_edge_checks",
            "state_slots_allocated",
            "parent_id_bytes",
            "state_capacity_bytes_peak",
            "frontier_capacity_bytes_peak",
            "path_workspace_bytes_peak",
            "result_path_bytes",
            "query_workspace_peak_bytes",
            "native_requested_bytes_peak",
        )
    ]

    def __init__(self) -> None:
        super().__init__()
        self.struct_size = ctypes.sizeof(type(self))


def copy_trace_result(
    result: TraceResult,
    *,
    kind: Literal["discrete", "continuous"],
    node_labels: Sequence[Any] | None = None,
    graph_bytes: int = 0,
) -> PlannerTrace:
    """Copy native-owned trace arrays before calling the matching free function."""
    if ctypes.sizeof(TraceEvent) != TRACE_EVENT_DTYPE.itemsize:
        raise RuntimeError("native trace event layout does not match NumPy")
    events = (
        np.ctypeslib.as_array(result.events, shape=(int(result.event_count),))
        .view(TRACE_EVENT_DTYPE)
        .copy()
        if result.event_count
        else np.empty(0, dtype=TRACE_EVENT_DTYPE)
    )
    points = None
    if result.point_count and result.dimension:
        points = (
            np.ctypeslib.as_array(
                result.points,
                shape=(int(result.point_count) * int(result.dimension),),
            )
            .copy()
            .reshape(int(result.point_count), int(result.dimension))
        )
    return PlannerTrace(
        kind=kind,
        events=events,
        points=points,
        node_labels=node_labels,
        truncated=bool(result.truncated),
        graph_bytes=graph_bytes,
    )


_NATIVE_LIB: ctypes.CDLL | None = None
_CONTINUOUS_NATIVE_LIB: ctypes.CDLL | None = None
_SEARCH_TRACE_LIB: ctypes.CDLL | None = None
_SEARCH_METRICS_LIB: ctypes.CDLL | None = None
_CONTINUOUS_TRACE_LIB: ctypes.CDLL | None = None

# Keep these exact-match requirements synchronized with abi_version.h.
_SEARCH_ABI_VERSION = 15
_SEARCH_METRICS_ABI_VERSION = 2
_CONTINUOUS_ABI_VERSION = 2
_SEARCH_TRACE_ABI_VERSION = 10
_CONTINUOUS_TRACE_ABI_VERSION = 2


class NativeLibraryLoadError(RuntimeError):
    """Raised when a packaged native engine cannot be loaded or used."""


class NativeLibraryCompatibilityError(NativeLibraryLoadError):
    """Raised when a native engine does not implement the expected C ABI."""


def _load_compatible_library(
    path: Path,
    *,
    engine: str,
    abi_probe_name: str,
    expected_abi: int,
    required_symbols: tuple[str, ...],
) -> ctypes.CDLL:
    try:
        library = ctypes.CDLL(str(path))
    except OSError as exc:
        raise NativeLibraryLoadError(
            f"could not load native {engine} library at {path}: {exc}. "
            "Rebuild or reinstall PathPlanning with `make build-ext`."
        ) from exc

    try:
        abi_probe = getattr(library, abi_probe_name)
    except AttributeError as exc:
        raise NativeLibraryCompatibilityError(
            f"native {engine} library at {path} does not export {abi_probe_name}; "
            f"expected ABI v{expected_abi}. Rebuild or reinstall PathPlanning "
            "with `make build-ext`."
        ) from exc

    abi_probe.argtypes = []
    abi_probe.restype = ctypes.c_uint32
    try:
        actual_abi = int(abi_probe())
    except (OSError, TypeError, ValueError) as exc:
        raise NativeLibraryCompatibilityError(
            f"could not read ABI version from native {engine} library at {path}; "
            f"expected ABI v{expected_abi}. Rebuild or reinstall PathPlanning "
            "with `make build-ext`."
        ) from exc
    if actual_abi != expected_abi:
        raise NativeLibraryCompatibilityError(
            f"Python requires native {engine} ABI v{expected_abi}, but loaded ABI "
            f"v{actual_abi} from {path}. Rebuild or reinstall PathPlanning "
            "with `make build-ext`."
        )

    missing_symbols = [name for name in required_symbols if not hasattr(library, name)]
    if missing_symbols:
        symbols = ", ".join(missing_symbols)
        raise NativeLibraryCompatibilityError(
            f"native {engine} library at {path} reports ABI v{actual_abi} but is "
            f"missing required exports: {symbols}. Rebuild or reinstall "
            "PathPlanning with `make build-ext`."
        )
    return library


def native_directory() -> Path:
    """Return the package directory containing the compiled extension."""
    return Path(__file__).resolve().parent


def _configure_graph_storage_functions(library: ctypes.CDLL) -> None:
    library.pp_graph_prepare_reverse.argtypes = [
        ctypes.c_void_p,
        ctypes.POINTER(ctypes.c_char),
        ctypes.c_size_t,
    ]
    library.pp_graph_prepare_reverse.restype = ctypes.c_int
    library.pp_graph_get_storage_info.argtypes = [
        ctypes.c_void_p,
        ctypes.POINTER(GraphStorageInfo),
    ]
    library.pp_graph_get_storage_info.restype = ctypes.c_int


def _configure_sipp_function(library: ctypes.CDLL, name: str) -> None:
    pointer_u64 = ctypes.POINTER(ctypes.c_uint64)
    pointer_double = ctypes.POINTER(ctypes.c_double)
    arguments: list[Any] = [
        ctypes.c_uint64,
        ctypes.c_uint64,
        pointer_u64,
        pointer_u64,
        pointer_double,
        pointer_u64,
        pointer_double,
        pointer_double,
        ctypes.c_uint64,
        pointer_u64,
        pointer_double,
        pointer_double,
        ctypes.c_uint64,
        ctypes.c_uint64,
        ctypes.c_uint64,
        ctypes.c_double,
        ctypes.c_int,
        ctypes.c_uint64,
        ctypes.c_double,
    ]
    is_bounded = name in {"pp_bounded_sipp_plan", "pp_bounded_sipp_plan_traced"}
    is_traced = name in {"pp_sipp_plan_traced", "pp_bounded_sipp_plan_traced"}
    if is_bounded:
        arguments.append(ctypes.c_double)
    if is_traced:
        arguments.append(ctypes.c_uint64)
    arguments.append(ctypes.POINTER(SippResult))
    if is_traced:
        arguments.append(ctypes.POINTER(TraceResult))
    function = getattr(library, name)
    function.argtypes = arguments
    function.restype = ctypes.c_int
    library.pp_sipp_free_result.argtypes = [ctypes.POINTER(SippResult)]
    library.pp_sipp_free_result.restype = None


def _configure_kinodynamic_sipp_function(library: ctypes.CDLL, name: str) -> None:
    pointer_u64 = ctypes.POINTER(ctypes.c_uint64)
    pointer_u8 = ctypes.POINTER(ctypes.c_uint8)
    pointer_double = ctypes.POINTER(ctypes.c_double)
    arguments: list[Any] = [
        ctypes.c_uint64,
        ctypes.c_uint64,
        pointer_u64,
        pointer_u64,
        pointer_u8,
        pointer_double,
        pointer_u64,
        pointer_double,
        pointer_double,
        ctypes.c_uint64,
        pointer_u64,
        pointer_double,
        pointer_double,
        ctypes.c_uint64,
        ctypes.c_uint64,
        ctypes.c_uint64,
        ctypes.c_double,
        ctypes.c_int,
        ctypes.c_uint64,
        ctypes.c_double,
    ]
    is_traced = name == "pp_kinodynamic_sipp_plan_traced"
    if is_traced:
        arguments.append(ctypes.c_uint64)
    arguments.append(ctypes.POINTER(SippResult))
    if is_traced:
        arguments.append(ctypes.POINTER(TraceResult))
    function = getattr(library, name)
    function.argtypes = arguments
    function.restype = ctypes.c_int


def _configure_eecbs_function(library: ctypes.CDLL, name: str) -> None:
    pointer_u64 = ctypes.POINTER(ctypes.c_uint64)
    arguments: list[Any] = [
        ctypes.c_uint64,
        ctypes.c_uint64,
        pointer_u64,
        pointer_u64,
        pointer_u64,
        pointer_u64,
        ctypes.c_size_t,
        ctypes.c_double,
        ctypes.c_int,
        ctypes.c_uint64,
        ctypes.c_double,
    ]
    is_traced = name == "pp_eecbs_plan_traced"
    if is_traced:
        arguments.append(ctypes.c_uint64)
    arguments.append(ctypes.POINTER(MapfResult))
    if is_traced:
        arguments.append(ctypes.POINTER(TraceResult))
    function = getattr(library, name)
    function.argtypes = arguments
    function.restype = ctypes.c_int
    library.pp_mapf_free_result.argtypes = [ctypes.POINTER(MapfResult)]
    library.pp_mapf_free_result.restype = None


def _configure_lacam_star_function(library: ctypes.CDLL, name: str) -> None:
    pointer_u64 = ctypes.POINTER(ctypes.c_uint64)
    arguments: list[Any] = [
        ctypes.c_uint64,
        ctypes.c_uint64,
        pointer_u64,
        pointer_u64,
        pointer_u64,
        pointer_u64,
        ctypes.c_size_t,
        ctypes.c_uint64,
        ctypes.c_int,
        ctypes.c_uint64,
        ctypes.c_double,
    ]
    is_traced = name == "pp_lacam_star_plan_traced"
    if is_traced:
        arguments.append(ctypes.c_uint64)
    arguments.append(ctypes.POINTER(MapfResult))
    if is_traced:
        arguments.append(ctypes.POINTER(TraceResult))
    function = getattr(library, name)
    function.argtypes = arguments
    function.restype = ctypes.c_int
    library.pp_mapf_free_result.argtypes = [ctypes.POINTER(MapfResult)]
    library.pp_mapf_free_result.restype = None


def load_native_library() -> ctypes.CDLL:
    """Load and configure the compiled C++ graph-search library once per process."""
    global _NATIVE_LIB
    if _NATIVE_LIB is not None:
        return _NATIVE_LIB

    candidates = [native_directory() / f"_search_engine{suffix}" for suffix in EXTENSION_SUFFIXES]
    for candidate in candidates:
        if not candidate.exists():
            continue
        library = _load_compatible_library(
            candidate,
            engine="search",
            abi_probe_name="pp_search_abi_version",
            expected_abi=_SEARCH_ABI_VERSION,
            required_symbols=(
                "pp_graph_create_csr",
                "pp_graph_create_grid",
                "pp_graph_create_grid_ex",
                "pp_graph_export_csr_view",
                "pp_graph_prepare_reverse",
                "pp_graph_get_storage_info",
                "pp_graph_free",
                "pp_native_search_plan",
                "pp_native_jps_grid",
                "pp_native_jpsw_grid",
                "pp_native_theta_star_grid",
                "pp_native_lazy_theta_star_grid",
                "pp_dstar_lite_create",
                "pp_dstar_lite_free",
                "pp_dstar_lite_plan",
                "pp_dstar_lite_plan_traced",
                "pp_dstar_lite_move_start",
                "pp_dstar_lite_update_edges",
                "pp_dstar_lite_reset",
                "pp_dstar_lite_trace_free_result",
                "pp_sipp_plan",
                "pp_bounded_sipp_plan",
                "pp_kinodynamic_sipp_plan",
                "pp_sipp_free_result",
                "pp_eecbs_plan",
                "pp_lacam_star_plan",
                "pp_mapf_free_result",
                "pp_search_free_result",
                "pp_search_engine_version",
            ),
        )
        library.pp_graph_create_csr.argtypes = [
            ctypes.c_uint64,
            ctypes.c_uint64,
            ctypes.POINTER(ctypes.c_uint64),
            ctypes.POINTER(ctypes.c_uint64),
            ctypes.POINTER(ctypes.c_double),
            ctypes.POINTER(ctypes.c_void_p),
            ctypes.POINTER(ctypes.c_char),
            ctypes.c_size_t,
        ]
        library.pp_graph_create_csr.restype = ctypes.c_int
        library.pp_graph_create_grid_ex.argtypes = [
            ctypes.c_uint64,
            ctypes.c_uint64,
            ctypes.c_uint64,
            ctypes.c_uint64,
            ctypes.c_int,
            ctypes.POINTER(ctypes.c_int32),
            ctypes.c_size_t,
            ctypes.POINTER(ctypes.c_uint8),
            ctypes.POINTER(ctypes.c_void_p),
            ctypes.POINTER(ctypes.c_char),
            ctypes.c_size_t,
        ]
        library.pp_graph_create_grid_ex.restype = ctypes.c_int
        library.pp_graph_free.argtypes = [ctypes.c_void_p]
        library.pp_graph_free.restype = None
        library.pp_graph_export_csr_view.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(GraphCsrView),
        ]
        library.pp_graph_export_csr_view.restype = ctypes.c_int
        _configure_graph_storage_functions(library)
        library.pp_native_search_plan.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(ctypes.c_uint8),
            ctypes.POINTER(ctypes.c_double),
            ctypes.c_uint64,
            ctypes.POINTER(SearchOptions),
            ctypes.POINTER(SearchResult),
        ]
        library.pp_native_search_plan.restype = ctypes.c_int
        library.pp_native_jps_grid.argtypes = [
            ctypes.c_uint64,
            ctypes.c_uint64,
            ctypes.POINTER(ctypes.c_uint8),
            ctypes.c_uint64,
            ctypes.c_uint64,
            ctypes.c_uint64,
            ctypes.c_uint64,
            ctypes.c_int,
            ctypes.c_uint64,
            ctypes.POINTER(SearchResult),
            ctypes.POINTER(JpsMetrics),
        ]
        library.pp_native_jps_grid.restype = ctypes.c_int
        library.pp_native_jpsw_grid.argtypes = [
            ctypes.c_uint64,
            ctypes.c_uint64,
            ctypes.POINTER(ctypes.c_uint8),
            ctypes.POINTER(ctypes.c_double),
            ctypes.c_uint64,
            ctypes.c_uint64,
            ctypes.c_uint64,
            ctypes.c_uint64,
            ctypes.c_int,
            ctypes.c_uint64,
            ctypes.POINTER(SearchResult),
            ctypes.POINTER(JpswMetrics),
        ]
        library.pp_native_jpsw_grid.restype = ctypes.c_int
        library.pp_native_theta_star_grid.argtypes = [
            ctypes.c_uint64,
            ctypes.c_uint64,
            ctypes.POINTER(ctypes.c_uint8),
            ctypes.POINTER(ctypes.c_double),
            ctypes.c_uint64,
            ctypes.c_uint64,
            ctypes.c_uint64,
            ctypes.c_uint64,
            ctypes.c_int,
            ctypes.c_uint64,
            ctypes.POINTER(SearchResult),
            ctypes.POINTER(ThetaMetrics),
        ]
        library.pp_native_theta_star_grid.restype = ctypes.c_int
        library.pp_native_lazy_theta_star_grid.argtypes = [
            ctypes.c_uint64,
            ctypes.c_uint64,
            ctypes.POINTER(ctypes.c_uint8),
            ctypes.POINTER(ctypes.c_double),
            ctypes.c_uint64,
            ctypes.c_uint64,
            ctypes.c_uint64,
            ctypes.c_uint64,
            ctypes.c_int,
            ctypes.c_uint64,
            ctypes.POINTER(SearchResult),
            ctypes.POINTER(ThetaMetrics),
        ]
        library.pp_native_lazy_theta_star_grid.restype = ctypes.c_int
        library.pp_dstar_lite_create.argtypes = [
            ctypes.c_uint64,
            ctypes.c_uint64,
            ctypes.POINTER(ctypes.c_uint64),
            ctypes.POINTER(ctypes.c_uint64),
            ctypes.POINTER(ctypes.c_double),
            ctypes.c_uint64,
            ctypes.c_uint64,
            ctypes.POINTER(ctypes.c_double),
            ctypes.POINTER(ctypes.c_void_p),
            ctypes.POINTER(ctypes.c_char),
            ctypes.c_size_t,
        ]
        library.pp_dstar_lite_create.restype = ctypes.c_int
        library.pp_dstar_lite_free.argtypes = [ctypes.c_void_p]
        library.pp_dstar_lite_free.restype = None
        library.pp_dstar_lite_plan.argtypes = [
            ctypes.c_void_p,
            ctypes.c_int,
            ctypes.c_uint64,
            ctypes.c_double,
            ctypes.POINTER(SearchResult),
        ]
        library.pp_dstar_lite_plan.restype = ctypes.c_int
        library.pp_dstar_lite_plan_traced.argtypes = [
            ctypes.c_void_p,
            ctypes.c_int,
            ctypes.c_uint64,
            ctypes.c_double,
            ctypes.c_uint64,
            ctypes.POINTER(SearchResult),
            ctypes.POINTER(TraceResult),
        ]
        library.pp_dstar_lite_plan_traced.restype = ctypes.c_int
        library.pp_dstar_lite_move_start.argtypes = [
            ctypes.c_void_p,
            ctypes.c_uint64,
            ctypes.POINTER(ctypes.c_double),
            ctypes.c_double,
            ctypes.POINTER(ctypes.c_char),
            ctypes.c_size_t,
        ]
        library.pp_dstar_lite_move_start.restype = ctypes.c_int
        library.pp_dstar_lite_update_edges.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(DStarEdgeUpdate),
            ctypes.c_size_t,
            ctypes.POINTER(ctypes.c_char),
            ctypes.c_size_t,
        ]
        library.pp_dstar_lite_update_edges.restype = ctypes.c_int
        library.pp_dstar_lite_reset.argtypes = [
            ctypes.c_void_p,
            ctypes.c_uint64,
            ctypes.c_uint64,
            ctypes.POINTER(ctypes.c_double),
            ctypes.POINTER(ctypes.c_char),
            ctypes.c_size_t,
        ]
        library.pp_dstar_lite_reset.restype = ctypes.c_int
        library.pp_dstar_lite_trace_free_result.argtypes = [ctypes.POINTER(TraceResult)]
        library.pp_dstar_lite_trace_free_result.restype = None
        _configure_sipp_function(library, "pp_sipp_plan")
        _configure_sipp_function(library, "pp_bounded_sipp_plan")
        _configure_kinodynamic_sipp_function(library, "pp_kinodynamic_sipp_plan")
        _configure_eecbs_function(library, "pp_eecbs_plan")
        _configure_lacam_star_function(library, "pp_lacam_star_plan")
        library.pp_search_free_result.argtypes = [ctypes.POINTER(SearchResult)]
        library.pp_search_free_result.restype = None
        library.pp_search_engine_version.argtypes = []
        library.pp_search_engine_version.restype = ctypes.c_char_p
        _NATIVE_LIB = library
        return library

    searched = ", ".join(str(path) for path in candidates)
    raise NativeLibraryLoadError(
        f"native search engine is not built; run `make build-ext`. Searched: {searched}"
    )


def load_continuous_library() -> ctypes.CDLL:
    """Load the compiled C sampling-planner library once per process."""
    global _CONTINUOUS_NATIVE_LIB
    if _CONTINUOUS_NATIVE_LIB is not None:
        return _CONTINUOUS_NATIVE_LIB

    candidates = [
        native_directory() / f"_continuous_engine{suffix}" for suffix in EXTENSION_SUFFIXES
    ]
    for candidate in candidates:
        if not candidate.exists():
            continue
        library = _load_compatible_library(
            candidate,
            engine="continuous",
            abi_probe_name="pp_continuous_abi_version",
            expected_abi=_CONTINUOUS_ABI_VERSION,
            required_symbols=(
                "pp_continuous_plan",
                "pp_continuous_free_result",
                "pp_dynamic_rrt_plan",
                "pp_dynamic_rrt_free_result",
                "pp_prm_star_create",
                "pp_prm_star_build",
                "pp_prm_star_query",
                "pp_prm_star_clear_query",
                "pp_prm_star_reset",
                "pp_prm_star_free",
                "pp_continuous_engine_version",
            ),
        )
        library.pp_continuous_engine_version.argtypes = []
        library.pp_continuous_engine_version.restype = ctypes.c_char_p
        _CONTINUOUS_NATIVE_LIB = library
        return library

    searched = ", ".join(str(path) for path in candidates)
    raise NativeLibraryLoadError(
        f"native continuous engine is not built; run `make build-ext`. Searched: {searched}"
    )


def _load_trace_library(
    module: str,
    engine: str,
    probe: str,
    expected_abi: int,
    symbols: tuple[str, ...],
) -> ctypes.CDLL:
    candidates = [native_directory() / f"{module}{suffix}" for suffix in EXTENSION_SUFFIXES]
    for candidate in candidates:
        if candidate.exists():
            return _load_compatible_library(
                candidate,
                engine=f"{engine} trace",
                abi_probe_name=probe,
                expected_abi=expected_abi,
                required_symbols=symbols,
            )
    raise NativeLibraryLoadError(
        f"native {engine} trace engine is not built; run `make build-ext`. "
        f"Searched: {', '.join(map(str, candidates))}"
    )


def load_search_trace_library() -> ctypes.CDLL:
    """Load the diagnostic C++ library only when trace recording is requested."""
    global _SEARCH_TRACE_LIB
    if _SEARCH_TRACE_LIB is None:
        _SEARCH_TRACE_LIB = _load_trace_library(
            "_search_trace_engine",
            "search",
            "pp_search_trace_abi_version",
            _SEARCH_TRACE_ABI_VERSION,
            (
                "pp_graph_create_csr_view",
                "pp_graph_storage_bytes",
                "pp_graph_free",
                "pp_native_search_plan_traced",
                "pp_native_jps_grid_traced",
                "pp_native_jpsw_grid_traced",
                "pp_native_theta_star_grid_traced",
                "pp_native_lazy_theta_star_grid_traced",
                "pp_sipp_plan_traced",
                "pp_bounded_sipp_plan_traced",
                "pp_kinodynamic_sipp_plan_traced",
                "pp_eecbs_plan_traced",
                "pp_lacam_star_plan_traced",
                "pp_sipp_free_result",
                "pp_search_free_result",
                "pp_search_trace_free_result",
            ),
        )
        _SEARCH_TRACE_LIB.pp_graph_create_csr_view.argtypes = [
            ctypes.POINTER(GraphCsrView),
            ctypes.POINTER(ctypes.c_void_p),
            ctypes.POINTER(ctypes.c_char),
            ctypes.c_size_t,
        ]
        _SEARCH_TRACE_LIB.pp_graph_create_csr_view.restype = ctypes.c_int
        _SEARCH_TRACE_LIB.pp_graph_free.argtypes = [ctypes.c_void_p]
        _SEARCH_TRACE_LIB.pp_graph_free.restype = None
        _SEARCH_TRACE_LIB.pp_graph_storage_bytes.argtypes = [ctypes.c_void_p]
        _SEARCH_TRACE_LIB.pp_graph_storage_bytes.restype = ctypes.c_uint64
        _SEARCH_TRACE_LIB.pp_native_search_plan_traced.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(ctypes.c_uint8),
            ctypes.POINTER(ctypes.c_double),
            ctypes.c_uint64,
            ctypes.POINTER(SearchOptions),
            ctypes.POINTER(SearchResult),
            ctypes.c_uint64,
            ctypes.POINTER(TraceResult),
        ]
        _SEARCH_TRACE_LIB.pp_native_search_plan_traced.restype = ctypes.c_int
        _SEARCH_TRACE_LIB.pp_native_jps_grid_traced.argtypes = [
            ctypes.c_uint64,
            ctypes.c_uint64,
            ctypes.POINTER(ctypes.c_uint8),
            ctypes.c_uint64,
            ctypes.c_uint64,
            ctypes.c_uint64,
            ctypes.c_uint64,
            ctypes.c_int,
            ctypes.c_uint64,
            ctypes.c_uint64,
            ctypes.POINTER(SearchResult),
            ctypes.POINTER(TraceResult),
            ctypes.POINTER(JpsMetrics),
        ]
        _SEARCH_TRACE_LIB.pp_native_jps_grid_traced.restype = ctypes.c_int
        _SEARCH_TRACE_LIB.pp_native_jpsw_grid_traced.argtypes = [
            ctypes.c_uint64,
            ctypes.c_uint64,
            ctypes.POINTER(ctypes.c_uint8),
            ctypes.POINTER(ctypes.c_double),
            ctypes.c_uint64,
            ctypes.c_uint64,
            ctypes.c_uint64,
            ctypes.c_uint64,
            ctypes.c_int,
            ctypes.c_uint64,
            ctypes.c_uint64,
            ctypes.POINTER(SearchResult),
            ctypes.POINTER(TraceResult),
            ctypes.POINTER(JpswMetrics),
        ]
        _SEARCH_TRACE_LIB.pp_native_jpsw_grid_traced.restype = ctypes.c_int
        _SEARCH_TRACE_LIB.pp_native_theta_star_grid_traced.argtypes = [
            ctypes.c_uint64,
            ctypes.c_uint64,
            ctypes.POINTER(ctypes.c_uint8),
            ctypes.POINTER(ctypes.c_double),
            ctypes.c_uint64,
            ctypes.c_uint64,
            ctypes.c_uint64,
            ctypes.c_uint64,
            ctypes.c_int,
            ctypes.c_uint64,
            ctypes.c_uint64,
            ctypes.POINTER(SearchResult),
            ctypes.POINTER(TraceResult),
            ctypes.POINTER(ThetaMetrics),
        ]
        _SEARCH_TRACE_LIB.pp_native_theta_star_grid_traced.restype = ctypes.c_int
        _SEARCH_TRACE_LIB.pp_native_lazy_theta_star_grid_traced.argtypes = [
            ctypes.c_uint64,
            ctypes.c_uint64,
            ctypes.POINTER(ctypes.c_uint8),
            ctypes.POINTER(ctypes.c_double),
            ctypes.c_uint64,
            ctypes.c_uint64,
            ctypes.c_uint64,
            ctypes.c_uint64,
            ctypes.c_int,
            ctypes.c_uint64,
            ctypes.c_uint64,
            ctypes.POINTER(SearchResult),
            ctypes.POINTER(TraceResult),
            ctypes.POINTER(ThetaMetrics),
        ]
        _SEARCH_TRACE_LIB.pp_native_lazy_theta_star_grid_traced.restype = ctypes.c_int
        _configure_sipp_function(_SEARCH_TRACE_LIB, "pp_sipp_plan_traced")
        _configure_sipp_function(_SEARCH_TRACE_LIB, "pp_bounded_sipp_plan_traced")
        _configure_kinodynamic_sipp_function(_SEARCH_TRACE_LIB, "pp_kinodynamic_sipp_plan_traced")
        _configure_eecbs_function(_SEARCH_TRACE_LIB, "pp_eecbs_plan_traced")
        _configure_lacam_star_function(_SEARCH_TRACE_LIB, "pp_lacam_star_plan_traced")
        _SEARCH_TRACE_LIB.pp_search_free_result.argtypes = [ctypes.POINTER(SearchResult)]
        _SEARCH_TRACE_LIB.pp_search_free_result.restype = None
        _SEARCH_TRACE_LIB.pp_search_trace_free_result.argtypes = [ctypes.POINTER(TraceResult)]
        _SEARCH_TRACE_LIB.pp_search_trace_free_result.restype = None
    return _SEARCH_TRACE_LIB


def load_search_metrics_library() -> ctypes.CDLL:
    """Load the isolated work/memory engine only for diagnostic observations."""
    global _SEARCH_METRICS_LIB
    if _SEARCH_METRICS_LIB is not None:
        return _SEARCH_METRICS_LIB

    candidates = [
        native_directory() / f"_search_metrics_engine{suffix}" for suffix in EXTENSION_SUFFIXES
    ]
    for candidate in candidates:
        if not candidate.exists():
            continue
        library = _load_compatible_library(
            candidate,
            engine="search metrics",
            abi_probe_name="pp_search_metrics_abi_version",
            expected_abi=_SEARCH_METRICS_ABI_VERSION,
            required_symbols=(
                "pp_search_abi_version",
                "pp_search_metrics_struct_size",
                "pp_graph_create_csr_view",
                "pp_graph_export_csr_view",
                "pp_graph_prepare_reverse",
                "pp_graph_get_storage_info",
                "pp_graph_free",
                "pp_native_search_plan_measured",
                "pp_search_free_result",
            ),
        )
        library.pp_search_abi_version.argtypes = []
        library.pp_search_abi_version.restype = ctypes.c_uint32
        if library.pp_search_abi_version() != _SEARCH_ABI_VERSION:
            raise NativeLibraryCompatibilityError(
                f"native search metrics library at {candidate} has a mismatched search ABI; "
                "rebuild with `make build-ext`."
            )
        library.pp_search_metrics_struct_size.argtypes = []
        library.pp_search_metrics_struct_size.restype = ctypes.c_uint64
        if library.pp_search_metrics_struct_size() != ctypes.sizeof(SearchMetrics):
            raise NativeLibraryCompatibilityError(
                f"native search metrics layout at {candidate} does not match Python; "
                "rebuild with `make build-ext`."
            )
        library.pp_graph_create_csr_view.argtypes = [
            ctypes.POINTER(GraphCsrView),
            ctypes.POINTER(ctypes.c_void_p),
            ctypes.POINTER(ctypes.c_char),
            ctypes.c_size_t,
        ]
        library.pp_graph_create_csr_view.restype = ctypes.c_int
        library.pp_graph_export_csr_view.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(GraphCsrView),
        ]
        library.pp_graph_export_csr_view.restype = ctypes.c_int
        _configure_graph_storage_functions(library)
        library.pp_graph_free.argtypes = [ctypes.c_void_p]
        library.pp_graph_free.restype = None
        library.pp_native_search_plan_measured.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(ctypes.c_uint8),
            ctypes.POINTER(ctypes.c_double),
            ctypes.c_uint64,
            ctypes.POINTER(SearchOptions),
            ctypes.POINTER(SearchResult),
            ctypes.POINTER(SearchMetrics),
        ]
        library.pp_native_search_plan_measured.restype = ctypes.c_int
        library.pp_search_free_result.argtypes = [ctypes.POINTER(SearchResult)]
        library.pp_search_free_result.restype = None
        _SEARCH_METRICS_LIB = library
        return library

    raise NativeLibraryLoadError(
        "native search metrics engine is not built; run `make build-ext`. "
        f"Searched: {', '.join(map(str, candidates))}"
    )


def load_continuous_trace_library() -> ctypes.CDLL:
    """Load the diagnostic C library only when trace recording is requested."""
    global _CONTINUOUS_TRACE_LIB
    if _CONTINUOUS_TRACE_LIB is None:
        _CONTINUOUS_TRACE_LIB = _load_trace_library(
            "_continuous_trace_engine",
            "continuous",
            "pp_continuous_trace_abi_version",
            _CONTINUOUS_TRACE_ABI_VERSION,
            (
                "pp_continuous_plan_traced",
                "pp_dynamic_rrt_plan_traced",
                "pp_prm_star_query_traced",
                "pp_continuous_free_result",
                "pp_dynamic_rrt_free_result",
                "pp_continuous_trace_free_result",
            ),
        )
    return _CONTINUOUS_TRACE_LIB


__all__ = [
    "NativeLibraryCompatibilityError",
    "NativeLibraryLoadError",
    "SearchOptions",
    "SearchResult",
    "GraphCsrView",
    "GraphStorageInfo",
    "SearchMetrics",
    "TraceEvent",
    "TraceResult",
    "copy_trace_result",
    "load_continuous_library",
    "load_continuous_trace_library",
    "load_native_library",
    "load_search_trace_library",
    "load_search_metrics_library",
]
