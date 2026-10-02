"""Private ctypes declarations for native search and sampling engines."""

from __future__ import annotations

import ctypes
from importlib.machinery import EXTENSION_SUFFIXES
from pathlib import Path


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


_NATIVE_LIB: ctypes.CDLL | None = None
_CONTINUOUS_NATIVE_LIB: ctypes.CDLL | None = None

# Keep these exact-match requirements synchronized with abi_version.h.
_SEARCH_ABI_VERSION = 1
_CONTINUOUS_ABI_VERSION = 1


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


__all__ = [
    "NativeLibraryCompatibilityError",
    "NativeLibraryLoadError",
    "SearchOptions",
    "SearchResult",
    "load_continuous_library",
    "load_native_library",
]
