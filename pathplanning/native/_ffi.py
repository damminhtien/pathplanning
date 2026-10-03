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
    ]


class SearchResult(ctypes.Structure):
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
_CONTINUOUS_TRACE_LIB: ctypes.CDLL | None = None

# Keep these exact-match requirements synchronized with abi_version.h.
_SEARCH_ABI_VERSION = 2
_CONTINUOUS_ABI_VERSION = 1
_TRACE_ABI_VERSION = 1


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
                "pp_graph_free",
                "pp_search_plan",
                "pp_native_search_plan",
                "pp_astar_plan",
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
        library.pp_native_search_plan.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(ctypes.c_uint8),
            ctypes.POINTER(ctypes.c_double),
            ctypes.c_uint64,
            ctypes.POINTER(SearchOptions),
            ctypes.POINTER(SearchResult),
        ]
        library.pp_native_search_plan.restype = ctypes.c_int
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
    symbols: tuple[str, ...],
) -> ctypes.CDLL:
    candidates = [native_directory() / f"{module}{suffix}" for suffix in EXTENSION_SUFFIXES]
    for candidate in candidates:
        if candidate.exists():
            return _load_compatible_library(
                candidate,
                engine=f"{engine} trace",
                abi_probe_name=probe,
                expected_abi=_TRACE_ABI_VERSION,
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
            (
                "pp_graph_create_csr_view",
                "pp_graph_storage_bytes",
                "pp_graph_free",
                "pp_native_search_plan_traced",
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
        _SEARCH_TRACE_LIB.pp_search_free_result.argtypes = [ctypes.POINTER(SearchResult)]
        _SEARCH_TRACE_LIB.pp_search_free_result.restype = None
        _SEARCH_TRACE_LIB.pp_search_trace_free_result.argtypes = [ctypes.POINTER(TraceResult)]
        _SEARCH_TRACE_LIB.pp_search_trace_free_result.restype = None
    return _SEARCH_TRACE_LIB


def load_continuous_trace_library() -> ctypes.CDLL:
    """Load the diagnostic C library only when trace recording is requested."""
    global _CONTINUOUS_TRACE_LIB
    if _CONTINUOUS_TRACE_LIB is None:
        _CONTINUOUS_TRACE_LIB = _load_trace_library(
            "_continuous_trace_engine",
            "continuous",
            "pp_continuous_trace_abi_version",
            (
                "pp_continuous_plan_traced",
                "pp_dynamic_rrt_plan_traced",
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
    "TraceEvent",
    "TraceResult",
    "copy_trace_result",
    "load_continuous_library",
    "load_continuous_trace_library",
    "load_native_library",
    "load_search_trace_library",
]
