"""Private ctypes declarations for the native search engine."""

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


def native_directory() -> Path:
    """Return the package directory containing the compiled extension."""
    return Path(__file__).resolve().parent


def load_native_library() -> ctypes.CDLL:
    """Load and configure the compiled C ABI library once per process."""
    global _NATIVE_LIB
    if _NATIVE_LIB is not None:
        return _NATIVE_LIB

    candidates = [native_directory() / f"_search_engine{suffix}" for suffix in EXTENSION_SUFFIXES]
    for candidate in candidates:
        if not candidate.exists():
            continue
        library = ctypes.CDLL(str(candidate))
        try:
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
        except AttributeError as exc:
            raise RuntimeError(
                "native search engine is stale; run `make build-ext` to rebuild it"
            ) from exc
        _NATIVE_LIB = library
        return library

    searched = ", ".join(str(path) for path in candidates)
    raise RuntimeError(
        f"native search engine is not built; run `make build-ext`. Searched: {searched}"
    )


__all__ = ["SearchOptions", "SearchResult", "load_native_library"]
