"""Check denominator, pairing, censoring, and scaling analysis semantics."""

from __future__ import annotations

import json
import math

import pytest

from scripts.shortest_path_benchmark.analysis import (
    analyze_campaign,
    clustered_bootstrap_speedup,
    preprocessing_break_even,
    summarize_campaign,
)
from scripts.shortest_path_benchmark.scaling import (
    ablation_factor,
    density_sweep,
    fit_loglog_slope,
    heuristic_variants,
    octile_heuristic_array,
    size_sweep,
    topology_sweep,
)


def _run(
    workload: str, variant: str, *, time_s: float | None, reference_cost: float | None,
    path_cost: float | None, repeat: int = 0, status: str = "ok", family: str = "test",
    map_id: str = "map-1",
) -> dict[str, object]:
    return {
        "workload_id": workload,
        "variant_id": variant,
        "pass": "latency",
        "scope": "public_api",
        "graph_state": "reused_graph",
        "phase": "measured",
        "repetition": repeat,
        "status": status,
        "input": {"reference_cost": reference_cost, "family": family, "map_sha256": map_id},
        "outcome": {
            "execution_status": "completed" if status == "ok" else status,
            "planner_stop_reason": "success" if path_cost is not None else "no_progress",
            "path_present": path_cost is not None,
            "path_valid": path_cost is not None,
            "path_cost": path_cost,
            "path_hash": f"{variant}-{workload}" if path_cost is not None else None,
        },
        "timing": {"api_total_s": time_s},
    }


def test_coverage_and_paired_latency_exclude_timeout_without_changing_denominator(tmp_path):
    cases = [
        {"workload_id": "w1", "input": {"reference_cost": 10, "map_sha256": "map-1"}},
        {"workload_id": "w2", "input": {"reference_cost": 20, "map_sha256": "map-2"}},
        {"workload_id": "w3", "input": {"reference_cost": 0, "map_sha256": "map-1"}},
        {"workload_id": "w4", "input": {"reference_reachable": False, "map_sha256": "map-2"}},
    ]
    manifest = {"cases": cases, "cohorts": {"latency": {"workload_ids": ["w1", "w2", "w3", "w4"]}}}
    runs = []
    for repeat, elapsed in enumerate((2.0, 4.0, 6.0)):
        runs.append(_run("w1", "dijkstra", time_s=elapsed, reference_cost=10,
                         path_cost=10, repeat=repeat))
        runs.append(_run("w1", "weighted", time_s=elapsed / 2, reference_cost=10,
                         path_cost=11, repeat=repeat))
        runs.append(_run("w2", "dijkstra", time_s=10, reference_cost=20,
                         path_cost=20, repeat=repeat, map_id="map-2"))
        runs.append(_run("w3", "dijkstra", time_s=1, reference_cost=0,
                         path_cost=0, repeat=repeat))
        runs.append(_run("w3", "weighted", time_s=0.5, reference_cost=0,
                         path_cost=0, repeat=repeat))
    runs.append(_run("w2", "weighted", time_s=5, reference_cost=20,
                     path_cost=None, status="timeout", map_id="map-2"))
    for variant in ("dijkstra", "weighted"):
        runs.append(_run("w4", variant, time_s=1, reference_cost=None,
                         path_cost=None, map_id="map-2"))
    summary = summarize_campaign(runs, manifest=manifest, bootstrap_draws=50)
    cohort = summary["cohorts"][0]
    assert cohort["eligibility"]["source"] == "manifest_cohort"
    baseline = cohort["variants"]["dijkstra"]
    weighted = cohort["variants"]["weighted"]
    assert baseline["coverage"]["valid_solved_count"] == 3
    assert weighted["coverage"]["valid_solved_count"] == 2
    assert weighted["coverage"]["oracle_solvable_count"] == 3
    assert weighted["coverage"]["unreachable_decision_accuracy"] == 1
    assert weighted["observations"]["timeouts"] == 1
    assert baseline["timing"]["query_medians_s"]["count"] == 3
    assert weighted["paired_vs_baseline"]["timed_pair_count"] == 2
    assert weighted["paired_vs_baseline"]["median_speedup"] == 2
    assert weighted["quality"]["zero_optimum_count"] == 1
    assert weighted["quality"]["mean_cost_ratio_denominator"] == 1
    assert weighted["quality"]["mean_cost_ratio"] == 1.1

    campaign = tmp_path / "campaign"
    campaign.mkdir()
    (campaign / "manifest.json").write_text(json.dumps(manifest), encoding="utf-8")
    (campaign / "runs.jsonl").write_text(
        "".join(json.dumps(row) + "\n" for row in runs), encoding="utf-8"
    )
    written = analyze_campaign(campaign, baseline="dijkstra", bootstrap_draws=50)
    assert written == summary
    assert (campaign / "summary.json").exists()
    assert "Valid / solvable" in (campaign / "report.md").read_text(encoding="utf-8")
    assert len(list((campaign / "plots").glob("*.svg"))) == 1


def test_report_includes_work_counters_and_separate_memory_boundaries(tmp_path):
    variants = ("dijkstra", "astar")
    manifest = {
        "cases": [
            {"workload_id": "w", "input": {"reference_cost": 10, "map_sha256": "map-1"}}
        ],
        "variants": [
            {"id": name, "variant_id": name, "variant_name": name} for name in variants
        ],
        "cohorts": {
            "work": {"workload_ids": ["w"]},
            "memory": {"workload_ids": ["w"]},
        },
    }
    runs = []
    for pass_name in ("work", "memory"):
        for index, variant in enumerate(variants):
            row = _run(
                "w",
                variant,
                time_s=None,
                reference_cost=10,
                path_cost=10,
            )
            row["pass"] = pass_name
            row["work"] = {
                "expanded": 10 + index,
                "edges_examined": 20 + index,
                "frontier_pushes": 8 + index,
                "frontier_pops": 7 + index,
                "stale_pops": 1,
                "frontier_peak_entries": 4 + index,
                "heuristic_lookups": 5,
                "heuristic_computations": 2,
            }
            row["memory"] = {
                "query_workspace_peak_bytes": 4 * 1024 * 1024 + index,
                "state_capacity_bytes_peak": 2 * 1024 * 1024,
                "frontier_capacity_bytes_peak": 1024 * 1024,
                "path_workspace_bytes_peak": 128 * 1024,
                "result_path_bytes": 64 * 1024,
                "process_peak_rss_bytes": (
                    100 * 1024 * 1024 + index if pass_name == "memory" else None
                ),
                "input_occupancy_bytes": 1024 * 1024 if pass_name == "memory" else 0,
                "base_csr_capacity_bytes_after": 8 * 1024 * 1024,
                "reverse_csr_capacity_bytes_after": 2 * 1024 * 1024,
                "worker_retained_graph_bytes": 8 * 1024 * 1024,
            }
            runs.append(row)

    summary = summarize_campaign(runs, manifest=manifest, bootstrap_draws=20)
    work = next(cohort for cohort in summary["cohorts"] if cohort["protocol"]["pass"] == "work")
    memory = next(
        cohort for cohort in summary["cohorts"] if cohort["protocol"]["pass"] == "memory"
    )
    assert work["variants"]["dijkstra"]["work"]["edges_examined"]["median"] == 20
    assert (
        work["variants"]["dijkstra"]["memory"]["query_workspace_peak_bytes"]["median"]
        == 4 * 1024 * 1024
    )
    assert (
        memory["variants"]["dijkstra"]["memory"]["process_peak_rss_bytes"]["median"]
        == 100 * 1024 * 1024
    )

    campaign = tmp_path / "campaign"
    campaign.mkdir()
    (campaign / "manifest.json").write_text(json.dumps(manifest), encoding="utf-8")
    (campaign / "runs.jsonl").write_text(
        "".join(json.dumps(row) + "\n" for row in runs), encoding="utf-8"
    )
    analyze_campaign(campaign, baseline="dijkstra", bootstrap_draws=20)
    report = (campaign / "report.md").read_text(encoding="utf-8")
    assert "### Search work per query (median / P95)" in report
    assert "Edges examined" in report
    assert "### Instrumented query memory (median / P95 MiB)" in report
    assert "### Fresh-worker memory (median / P95 MiB)" in report
    assert "Process RSS MiB" in report
    assert "not query-only memory" in report


def test_bootstrap_pairs_maps_and_is_repeatable():
    pairs = [("map-a", 2), ("map-a", 3), ("map-b", 1), ("map-c", 4)]
    first = clustered_bootstrap_speedup(pairs, draws=100, seed=7)
    assert first == clustered_bootstrap_speedup(pairs, draws=100, seed=7)
    assert first["map_count"] == 3
    assert first["few_maps"] is False
    assert first["median_ci95"][0] <= 2.5 <= first["median_ci95"][1]
    with pytest.raises(ValueError, match="positive"):
        clustered_bootstrap_speedup([("map", 0)])


def test_repeated_outcome_inconsistency_blocks_paired_speedup():
    runs = [
        _run("w", "dijkstra", time_s=2, reference_cost=1, path_cost=1),
        _run("w", "candidate", time_s=1, reference_cost=1, path_cost=1),
        _run("w", "candidate", time_s=1, reference_cost=1, path_cost=None,
             repeat=1, status="timeout"),
    ]
    result = summarize_campaign(runs, bootstrap_draws=10)["cohorts"][0]["variants"]["candidate"]
    assert result["observations"]["inconsistent_queries"] == ["w"]
    assert result["coverage"]["valid_solved_count"] == 0
    assert result["paired_vs_baseline"]["timed_pair_count"] == 0


def test_break_even_classifies_all_quadrants():
    assert preprocessing_break_even(1, 10, 6, 5)["first_query_count_b_wins"] == 1
    assert preprocessing_break_even(5, 10, 1, 5)["classification"] == "b_dominates"
    assert preprocessing_break_even(1, 5, 6, 10)["classification"] == "b_never_breaks_even"
    assert preprocessing_break_even(6, 5, 1, 10)["classification"] == "b_wins_below_threshold"
    with pytest.raises(ValueError, match="nonnegative"):
        preprocessing_break_even(-1, 1, 1, 1)


def test_scaling_cases_are_reproducible_and_preserve_sweep_identity():
    first = next(size_sweep(sizes=(64,), seeds=(7,), queries_per_bin=1))
    again = next(size_sweep(sizes=(64,), seeds=(7,), queries_per_bin=1))
    density = next(density_sweep(size=64, densities=(0.20,), seeds=(7,), queries_per_bin=1))
    assert first.manifest_record()["map_sha256"] == again.manifest_record()["map_sha256"]
    assert first.manifest_record()["map_sha256"] == density.manifest_record()["map_sha256"]
    assert len(first.queries) == 5
    assert {query.displacement_bin for query in first.queries} == set(range(5))
    assert all(not first.blocked[query.start[1], query.start[0]] for query in first.queries)
    assert all(not first.blocked[query.goal[1], query.goal[0]] for query in first.queries)
    assert first.connected_components >= 1


def test_topology_heuristic_slope_and_ablation_contract():
    cases = list(topology_sweep(size=64, opening_widths=(1,), seeds=(0,),
                                queries_per_bin=1))
    assert {case.topology for case in cases} == {"room", "serpentine_maze"}
    assert all(len(case.queries) == 5 for case in cases)
    assert [item["alpha"] for item in heuristic_variants()] == [0, 0.25, 0.5, 0.75, 1]
    h = octile_heuristic_array(3, (2, 2))
    assert h[0] == pytest.approx(2 * math.sqrt(2))
    assert h[-1] == 0
    assert not octile_heuristic_array(3, (2, 2), alpha=0).any()
    fit = fit_loglog_slope([(1, 1), (2, 4), (4, 16), (8, 64)], draws=100)
    assert fit["slope"] == pytest.approx(2)
    assert fit["interpretation"] == "empirical_trend_only"
    assert ablation_factor({"graph": "fresh", "h": "array"},
                           {"graph": "reused", "h": "array"}, factor="graph")["factor"] == "graph"
    with pytest.raises(ValueError, match="only graph"):
        ablation_factor({"graph": "fresh", "h": "array"},
                        {"graph": "reused", "h": "lazy"}, factor="graph")
