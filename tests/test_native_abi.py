"""Exact ABI checks for the production and diagnostic native libraries."""

from pathplanning.native import _ffi


def test_native_libraries_match_declared_abi_versions() -> None:
    search = _ffi.load_native_library()
    continuous = _ffi.load_continuous_library()
    metrics = _ffi.load_search_metrics_library()
    search_trace = _ffi.load_search_trace_library()
    continuous_trace = _ffi.load_continuous_trace_library()

    assert search.pp_search_abi_version() == _ffi._SEARCH_ABI_VERSION
    assert search.pp_kinodynamic_abi_version() == _ffi._KINODYNAMIC_ABI_VERSION
    assert metrics.pp_search_metrics_abi_version() == _ffi._SEARCH_METRICS_ABI_VERSION
    assert metrics.pp_search_abi_version() == _ffi._SEARCH_ABI_VERSION
    assert search_trace.pp_search_trace_abi_version() == _ffi._SEARCH_TRACE_ABI_VERSION
    assert search_trace.pp_kinodynamic_trace_abi_version() == _ffi._KINODYNAMIC_TRACE_ABI_VERSION
    assert continuous.pp_continuous_abi_version() == _ffi._CONTINUOUS_ABI_VERSION
    assert continuous_trace.pp_continuous_trace_abi_version() == _ffi._CONTINUOUS_TRACE_ABI_VERSION
