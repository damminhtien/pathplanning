"""Protocol invariants for the resumable shortest-path runner."""

from __future__ import annotations

import json

from pathplanning.api import plan_discrete
from pathplanning.core.contracts import DiscreteProblem
from pathplanning.spaces.grid2d import Grid2DSearchSpace
from scripts.shortest_path_benchmark.runner import (
    SCHEMA,
    VARIANTS,
    _complete_lines,
    _native_call,
    _outcome,
    _record_key,
    _schedule,
    _variant_params,
    prepare_manifest,
    validate_manifest,
)
from scripts.shortest_path_benchmark.workloads import load_map


def test_default_variants_cover_all_native_discrete_search_algorithms() -> None:
    assert {variant["algorithm"] for variant in VARIANTS} == set(range(1, 11))
    assert sum(variant["planner"] == "weighted_astar" for variant in VARIANTS) == 3

    graph = Grid2DSearchSpace(width=4, height=4)
    problem = DiscreteProblem(graph=graph, start=(0, 0), goal=(3, 3))
    for variant in VARIANTS:
        params = _variant_params(variant)
        result = plan_discrete(problem, planner=variant["planner"], params=params, seed=7)
        assert result.success, variant["id"]
    anytime = next(variant for variant in VARIANTS if variant["planner"] == "anytime_astar")
    assert _variant_params(anytime)["anytime_weights"] == (2.0, 1.5, 1.25, 1.0)


def _case(workload_id: str, map_path: str, bin_index: int) -> dict[str, object]:
    return {
        "workload_id": workload_id,
        "family": "fixture",
        "map_path": map_path,
        "map_sha256": "map-hash",
        "scenario_line": 2,
        "start": 0,
        "goal": 3,
        "scenario_optimum": "sqrt2",
        "reference_cost": 2**53 + 1,
        "node_slots": 4,
        "free_nodes": 4,
        "directed_edges": 12,
        "difficulty_bin": bin_index,
    }


def test_schedule_is_seeded_and_respects_latency_cohort() -> None:
    cases = [_case(f"w{i}", "tiny.map", i % 2) for i in range(3)]
    manifest = {
        "cases": cases,
        "variants": [
            {"id": "a", "variant_id": "variant-a", "variant_name": "astar", "planner": "astar"},
            {
                "id": "d",
                "variant_id": "variant-d",
                "variant_name": "dijkstra",
                "planner": "dijkstra",
            },
        ],
        "cohorts": {"latency": {"workload_ids": ["w0", "w2"]}},
    }
    first = _schedule(
        manifest,
        "latency",
        warmups=2,
        repeats=3,
        seed=7,
        scope="public_api",
        graph_state="reused_graph",
    )
    second = _schedule(
        manifest,
        "latency",
        warmups=2,
        repeats=3,
        seed=7,
        scope="public_api",
        graph_state="reused_graph",
    )
    assert first == second
    assert len(first) == 2 * (2 + 3) * 2
    assert {row["workload_id"] for row in first} == {"w0", "w2"}
    assert len({_record_key(row) for row in first}) == len(first)


def test_work_schedule_can_select_unreachable_cohort_without_pooling() -> None:
    cases = [
        _case("reachable", "map.map", 0),
        {**_case("unreachable", "map.map", 0), "query_kind": "unreachable"},
    ]
    manifest = {
        "cases": cases,
        "variants": [
            {"id": "d", "variant_id": "variant-d", "variant_name": "dijkstra"}
        ],
        "cohorts": {
            "work": {"workload_ids": ["reachable"]},
            "unreachable": {"workload_ids": ["unreachable"]},
        },
    }
    schedule = _schedule(
        manifest,
        "work",
        warmups=0,
        repeats=1,
        seed=7,
        scope="public_api",
        graph_state="reused_graph",
        cohort_name="unreachable",
    )
    assert {row["workload_id"] for row in schedule} == {"unreachable"}
    assert {row["cohort"] for row in schedule} == {"unreachable"}


def test_jsonl_recovery_preserves_large_integers_and_marks_partial_tail(tmp_path) -> None:
    path = tmp_path / "runs.jsonl"
    exact = 2**53 + 19
    complete = json.dumps({"counter": exact}, separators=(",", ":")).encode() + b"\n"
    path.write_bytes(complete + b'{"counter":')
    records, truncated, boundary = _complete_lines(path)
    assert records == [{"counter": exact}]
    assert truncated
    assert boundary == len(complete)


def test_outcome_uses_independent_path_cost_and_keeps_unreachable_distinct(tmp_path) -> None:
    map_path = tmp_path / "tiny.map"
    map_path.write_text("type octile\nheight 2\nwidth 2\nmap\n..\n..\n")
    grid = load_map(map_path)
    case = {
        "start": 0,
        "goal": 3,
        "reference_cost": 2**0.5,
    }
    observed = {
        "success": True,
        "stop_reason": 0,
        "iters": 2,
        "nodes": 3,
        "path_cost": 2**0.5,
        "path": [0, 3],
    }
    outcome = _outcome(grid, case, observed)
    assert outcome["execution_status"] == "valid_optimal"
    assert outcome["path_valid"] is True
    assert outcome["path_length"] == 2
    assert SCHEMA == "pathplanning_shortest_path_v2"

    unreachable = _outcome(
        grid,
        {"start": 0, "goal": 3, "reference_cost": None},
        {
            "success": False,
            "stop_reason": 1,
            "iters": 1,
            "nodes": 1,
            "path_cost": None,
            "path": None,
        },
    )
    assert unreachable["execution_status"] == "proved_unreachable"


def test_anytime_native_kernel_receives_its_weight_schedule(tmp_path) -> None:
    from pathplanning.native._ffi import load_native_library

    map_path = tmp_path / "tiny.map"
    map_path.write_text("type octile\nheight 4\nwidth 4\nmap\n....\n....\n....\n....\n")
    grid = load_map(map_path)
    graph = grid.to_native_graph()
    try:
        variant = next(variant for variant in VARIANTS if variant["planner"] == "anytime_astar")
        result = _native_call(
            load_native_library(),
            graph._native_handle,
            grid,
            {"start": 0, "goal": 15},
            variant,
        )
    finally:
        graph.close()
    assert result["success"]
    assert result["path_cost"] == 3 * 2**0.5


def test_scaling_grid_validation_uses_an_independent_reference_not_scenario_optima(
    tmp_path,
) -> None:
    from scripts.shortest_path_benchmark.scaling_manifest import prepare_scaling_manifest

    profile = {
        "profile": "scaling-test",
        "generator_version": "grid_sweep_v1",
        "size_sweep": {
            "side_lengths": [24],
            "obstacle_density": 0.2,
            "seeds": [1],
            "queries_per_seed": 5,
        },
        "density_sweep": {"side_length": 24, "densities": [0.2], "seeds": [1]},
        "heuristic_alpha": [],
    }
    profile_path = tmp_path / "profile.json"
    manifest_path = tmp_path / "scaling" / "manifest.json"
    profile_path.write_text(json.dumps(profile))
    prepare_scaling_manifest(profile_path, manifest_path)

    result = validate_manifest(manifest_path)
    assert result["checked"] == 5
    assert result["discrepancies"] == []


def test_coverage_profile_selects_distance_bins_and_keeps_memory_sample_small(tmp_path) -> None:
    dataset_root = tmp_path / "dataset"
    family = dataset_root / "family"
    family.mkdir(parents=True)
    (family / "map.map").write_text(
        "type octile\nheight 4\nwidth 4\nmap\n....\n....\n....\n....\n"
    )
    (family / "map.map.scen").write_text(
        "version 1\n"
        "0 map.map 4 4 0 0 1 0 1.000000000\n"
        "0 map.map 4 4 0 0 2 0 2.000000000\n"
        "0 map.map 4 4 0 0 3 0 3.000000000\n"
        "0 map.map 4 4 1 1 2 2 1.414213562\n"
    )

    manifest = prepare_manifest(
        dataset_root, tmp_path / "coverage.json", profile="coverage", seed=7
    )
    cases = manifest["cases"]
    assert manifest["selection"]["selection_version"] == "all_maps_displacement_bins_v1"
    assert len(cases) == 3
    assert len(manifest["cohorts"]["latency"]["workload_ids"]) == 1
    assert len(manifest["cohorts"]["memory"]["workload_ids"]) == 1
    assert {case["displacement_bin"] for case in cases} == {1, 2, 3}
