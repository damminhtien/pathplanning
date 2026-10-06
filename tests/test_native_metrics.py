"""The isolated metrics build reports exact work without changing search results."""

from __future__ import annotations

import ctypes

import numpy as np
import pytest

from pathplanning.native import NativeGraph
from pathplanning.native._ffi import (
    GraphCsrView,
    GraphStorageInfo,
    SearchMetrics,
    SearchOptions,
    SearchResult,
    load_native_library,
    load_search_metrics_library,
)


@pytest.fixture
def native_graph() -> NativeGraph[int]:
    # The 2->1 edge improves a queued label; goal 4 is unreachable.
    graph = NativeGraph.from_csr(
        np.array([0, 2, 3, 5, 5, 5], dtype=np.uint64),
        np.array([1, 2, 3, 1, 3], dtype=np.uint64),
        np.array([5.0, 1.0, 1.0, 1.0, 20.0], dtype=np.float64),
    )
    yield graph
    graph.close()


def _clone_for_metrics(graph: NativeGraph[int]) -> ctypes.c_void_p:
    release = load_native_library()
    metrics_lib = load_search_metrics_library()
    view = GraphCsrView()
    assert release.pp_graph_export_csr_view(graph._native_handle, ctypes.byref(view)) == 0
    clone = ctypes.c_void_p()
    error = ctypes.create_string_buffer(256)
    assert metrics_lib.pp_graph_create_csr_view(
        ctypes.byref(view), ctypes.byref(clone), error, len(error)
    ) == 0, error.value
    assert clone.value is not None
    return clone


def _run(
    library: ctypes.CDLL,
    graph: ctypes.c_void_p,
    algorithm: int,
    goal: int,
    heuristic: ctypes.Array[ctypes.c_double] | None = None,
    weights: ctypes.Array[ctypes.c_double] | None = None,
) -> tuple[tuple[int, int, int, int, float, tuple[int, ...]], SearchMetrics | None]:
    options = SearchOptions()
    options.algorithm = algorithm
    options.heuristic_weight = 1.0
    options.has_goal_id = 1
    options.goal_id = goal
    if weights is not None:
        options.anytime_weights = ctypes.cast(weights, ctypes.POINTER(ctypes.c_double))
        options.anytime_weight_count = len(weights)
    result = SearchResult()
    measured = library is load_search_metrics_library()
    counters = SearchMetrics() if measured else None
    try:
        if counters is None:
            status = library.pp_native_search_plan(
                graph, None, heuristic, 0, ctypes.byref(options), ctypes.byref(result)
            )
        else:
            status = library.pp_native_search_plan_measured(
                graph,
                None,
                heuristic,
                0,
                ctypes.byref(options),
                ctypes.byref(result),
                ctypes.byref(counters),
            )
        assert status == 0, result.error_message
        values = tuple(result.path_ids[index] for index in range(result.path_length))
        outcome = (
            result.success,
            result.stop_reason,
            result.iters,
            result.nodes,
            result.path_cost,
            values,
        )
        return outcome, counters
    finally:
        library.pp_search_free_result(ctypes.byref(result))


@pytest.mark.parametrize("algorithm", [1, 2, 3, 4, 5, 6, 7, 8, 9])
def test_metrics_result_matches_release(native_graph: NativeGraph[int], algorithm: int) -> None:
    release = load_native_library()
    metrics_lib = load_search_metrics_library()
    clone = _clone_for_metrics(native_graph)
    heuristic = (ctypes.c_double * 5)(3.0, 1.0, 2.0, 0.0, 0.0)
    weights = (ctypes.c_double * 3)(2.0, 1.5, 1.0)
    try:
        actual, counters = _run(
            metrics_lib, clone, algorithm, 3, heuristic, weights
        )
        expected, _ = _run(
            release, native_graph._native_handle, algorithm, 3, heuristic, weights
        )
        assert actual == expected
        assert counters is not None
        assert counters.expanded == actual[2]
        assert counters.discovered_first == actual[3]
        assert counters.frontier_pushes >= counters.frontier_pops
        assert counters.native_requested_bytes_peak >= counters.state_capacity_bytes_peak
        assert counters.capability_bits & 0b111 == 0b111
    finally:
        metrics_lib.pp_graph_free(clone)


def test_work_counters_include_stale_heap_entries(native_graph: NativeGraph[int]) -> None:
    library = load_search_metrics_library()
    clone = _clone_for_metrics(native_graph)
    try:
        outcome, counters = _run(library, clone, algorithm=5, goal=4)
        assert counters is not None
        assert outcome[:4] == (0, 2, 4, 4)
        assert counters.edges_examined == 5
        assert counters.relaxation_attempts == 5
        assert counters.relaxation_successes_first == 3
        assert counters.relaxation_successes_improved == 2
        assert counters.frontier_pushes == counters.frontier_pops == 6
        assert counters.stale_pops == 2
        assert counters.frontier_peak_entries == 3
        assert counters.state_slots_allocated == 5
        assert counters.parent_id_bytes == 4
        assert counters.state_capacity_bytes_peak == 5 * (8 + 4 + 1)
        assert counters.result_path_bytes == 0
    finally:
        library.pp_graph_free(clone)


def test_reverse_storage_is_prepared_once(native_graph: NativeGraph[int]) -> None:
    for library, handle in (
        (load_native_library(), native_graph._native_handle),
        (load_search_metrics_library(), _clone_for_metrics(native_graph)),
    ):
        try:
            before = GraphStorageInfo()
            after = GraphStorageInfo()
            repeated = GraphStorageInfo()
            assert library.pp_graph_get_storage_info(handle, ctypes.byref(before)) == 0
            assert before.node_count == 5 and before.edge_count == 5
            assert before.base_csr_capacity_bytes >= 6 * 8 + 5 * (8 + 8)
            assert before.reverse_csr_capacity_bytes == 0
            error = ctypes.create_string_buffer(256)
            assert library.pp_graph_prepare_reverse(handle, error, len(error)) == 0
            assert library.pp_graph_get_storage_info(handle, ctypes.byref(after)) == 0
            assert after.reverse_csr_capacity_bytes >= 6 * 8 + 5 * (8 + 8)
            assert library.pp_graph_prepare_reverse(handle, error, len(error)) == 0
            assert library.pp_graph_get_storage_info(handle, ctypes.byref(repeated)) == 0
            assert repeated.reverse_csr_capacity_bytes == after.reverse_csr_capacity_bytes
        finally:
            if library is load_search_metrics_library():
                library.pp_graph_free(handle)


def test_validation_work_is_separate_from_search_work(native_graph: NativeGraph[int]) -> None:
    library = load_search_metrics_library()
    clone = _clone_for_metrics(native_graph)
    heuristic = (ctypes.c_double * 5)(3.0, 1.0, 2.0, 0.0, 0.0)
    try:
        outcome, counters = _run(library, clone, algorithm=9, goal=3, heuristic=heuristic)
        assert outcome[0] == 1
        assert counters is not None
        assert counters.validation_edge_checks == 5
        assert counters.validation_heuristic_lookups == 10
        assert counters.heuristic_lookups > 0
        assert counters.edges_examined <= 5
    finally:
        library.pp_graph_free(clone)


def test_production_binary_has_no_metrics_entry(native_graph: NativeGraph[int]) -> None:
    release = load_native_library()
    assert not hasattr(release, "pp_native_search_plan_measured")
    assert not hasattr(release, "pp_search_metrics_abi_version")
    outcome, _ = _run(release, native_graph._native_handle, algorithm=5, goal=3)
    assert outcome[0] == 1
