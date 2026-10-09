from __future__ import annotations

import math

from scripts.benchmark_movingai_grid_specialists import (
    _any_angle_metrics,
    _grid_path_metrics,
    _read_jsonl,
    _run_one,
    _segment_visible,
)
from scripts.shortest_path_benchmark.workloads import load_map


def test_grid_specialist_path_validation_rejects_corner_cutting(tmp_path):
    map_path = tmp_path / "corner.map"
    map_path.write_text(
        "type octile\nheight 2\nwidth 2\nmap\n.@\n@.\n",
        encoding="ascii",
    )
    grid = load_map(map_path)

    assert not _grid_path_metrics(grid, [(0, 0), (1, 1)])[0]
    assert not _segment_visible(grid, (0.0, 0.0), (1.0, 1.0))


def test_grid_specialist_supercover_accepts_open_any_angle_segment(tmp_path):
    map_path = tmp_path / "open.map"
    map_path.write_text(
        "type octile\nheight 5\nwidth 5\nmap\n.....\n.....\n.....\n.....\n.....\n",
        encoding="ascii",
    )
    grid = load_map(map_path)

    valid, length = _any_angle_metrics(grid, [[0.0, 0.0], [4.0, 3.0]], (0, 0), (4, 3))

    assert valid
    assert math.isclose(length, 5.0)


def test_all_grid_specialists_run_through_public_api_on_small_land_map(tmp_path):
    map_path = tmp_path / "open.map"
    map_path.write_text(
        "type octile\nheight 5\nwidth 5\nmap\n.....\n.....\n.....\n.....\n.....\n",
        encoding="ascii",
    )
    grid = load_map(map_path)
    workload = {
        "workload_id": "small-workload",
        "map_path": "small/open.map",
        "map_sha256": grid.map_sha256,
        "scenario_path": "small/open.map.scen",
        "scenario_line": 1,
        "start": 0,
        "goal": 24,
        "scenario_optimum": "5.656854249",
        "displacement_bin": 4,
    }
    oracle = {"reference_cost": 4 * math.sqrt(2.0), "reference_reachable": True}

    rows = [
        _run_one(grid, workload, oracle, planner, seed=7)
        for planner in ("jps", "dstar_lite", "theta_star", "lazy_theta_star")
    ]

    assert all(row["outcome"]["status"] != "error" for row in rows)
    assert [row["outcome"]["status"] for row in rows] == [
        "valid_optimal",
        "valid_optimal",
        "valid_any_angle_path",
        "valid_any_angle_path",
    ]


def test_grid_specialist_jsonl_resume_discards_truncated_tail(tmp_path):
    path = tmp_path / "runs.jsonl"
    path.write_bytes(b'{"done":true}\n{"partial":')

    assert _read_jsonl(path) == [{"done": True}]
    assert path.read_bytes() == b'{"done":true}\n'
