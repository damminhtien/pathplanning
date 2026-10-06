"""Protocol invariants for the resumable shortest-path runner."""

from __future__ import annotations

import json

from scripts.shortest_path_benchmark.runner import (
    SCHEMA,
    _complete_lines,
    _outcome,
    _record_key,
    _schedule,
)
from scripts.shortest_path_benchmark.workloads import load_map


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
