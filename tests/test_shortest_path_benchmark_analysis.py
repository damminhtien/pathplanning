"""Check denominator, pairing, censoring, and scaling analysis semantics."""

from __future__ import annotations

import json

from scripts.shortest_path_benchmark.analysis import (
    analyze_campaign,
    summarize_campaign,
)


def _run(
    workload: str,
    variant: str,
    *,
    time_s: float | None,
    reference_cost: float | None,
    path_cost: float | None,
    repeat: int = 0,
    status: str = "ok",
    family: str = "test",
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
        runs.append(
            _run("w1", "dijkstra", time_s=elapsed, reference_cost=10, path_cost=10, repeat=repeat)
        )
        runs.append(
            _run(
                "w1", "weighted", time_s=elapsed / 2, reference_cost=10, path_cost=11, repeat=repeat
            )
        )
        runs.append(
            _run(
                "w2",
                "dijkstra",
                time_s=10,
                reference_cost=20,
                path_cost=20,
                repeat=repeat,
                map_id="map-2",
            )
        )
        runs.append(_run("w3", "dijkstra", time_s=1, reference_cost=0, path_cost=0, repeat=repeat))
        runs.append(
            _run("w3", "weighted", time_s=0.5, reference_cost=0, path_cost=0, repeat=repeat)
        )
    runs.append(
        _run(
            "w2",
            "weighted",
            time_s=5,
            reference_cost=20,
            path_cost=None,
            status="timeout",
            map_id="map-2",
        )
    )
    for variant in ("dijkstra", "weighted"):
        runs.append(
            _run("w4", variant, time_s=1, reference_cost=None, path_cost=None, map_id="map-2")
        )
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
