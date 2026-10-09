"""Boundary and representation checks for the strict MovingAI land profile."""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest

from scripts.shortest_path_benchmark.workloads import (
    DatasetError,
    load_map,
    make_case_record,
    parse_scenario,
    select_pilot_cases,
)


def _map(root: Path, name: str, rows: tuple[str, ...]) -> Path:
    path = root / name
    path.write_text(
        f"type octile\nheight {len(rows)}\nwidth {len(rows[0])}\nmap\n" + "\n".join(rows) + "\n"
    )
    return path


def _scenario(root: Path, rows: list[str], version: str = "version 1") -> Path:
    path = root / "queries.scen"
    path.write_text(version + "\n" + "\n".join(rows) + "\n")
    return path


def test_no_corner_cutting_csr_and_octile_values(tmp_path: Path) -> None:
    grid = load_map(_map(tmp_path, "open.map", ("..", "..")))
    offsets, indices, costs = grid.to_csr()
    assert grid.node_count == 4
    assert grid.free_count == 4
    assert grid.edge_count == 12
    assert offsets.tolist() == [0, 3, 6, 9, 12]
    assert indices[:3].tolist() == [2, 3, 1]
    assert costs[:3].tolist() == pytest.approx([1.0, math.sqrt(2), 1.0])
    assert grid.native_heuristic_values(3).tolist() == pytest.approx([math.sqrt(2), 1.0, 1.0, 0.0])
    assert all(not array.flags.writeable for array in (offsets, indices, costs))

    blocked = load_map(_map(tmp_path, "one-corner.map", (".@", "..")))
    offsets, indices, _ = blocked.to_csr()
    assert blocked.node_count == 4
    assert blocked.free_count == 3
    assert indices[offsets[0] : offsets[1]].tolist() == [2]
    assert offsets[1] == offsets[2]  # The blocked cell keeps its empty CSR row.


@pytest.mark.parametrize(
    ("rows", "code"),
    [((".?", ".."), "unknown_symbol"), (("..", "."), "dimension_mismatch")],
)
def test_map_rejects_unknown_or_wrong_size(
    tmp_path: Path, rows: tuple[str, ...], code: str
) -> None:
    with pytest.raises(DatasetError) as failure:
        load_map(_map(tmp_path, "bad.map", rows))
    assert failure.value.code == code


def test_scenario_resolves_each_map_and_preserves_optimum_text(tmp_path: Path) -> None:
    _map(tmp_path, "a.map", ("..", ".."))
    _map(tmp_path, "b.map", (".G", "S."))
    path = _scenario(
        tmp_path,
        [
            "0 a.map 2 2 0 0 1 1 1.414200",
            "1 b.map 2 2 0 1 1 0 1.4142",
        ],
        version="version 1.0",
    )
    scenarios = parse_scenario(path, tmp_path, family="dao")
    assert [row.map_name for row in scenarios] == ["a.map", "b.map"]
    assert scenarios[0].optimal_length_str == "1.414200"
    assert scenarios[1].start_id == 2
    case = make_case_record(scenarios[0], math.sqrt(2))
    assert case["map_path"] == "a.map"
    assert case["node_slots"] == 4
    assert case["directed_edges"] == 12
    assert case["workload_id"] == make_case_record(scenarios[0], 100.0)["workload_id"]


@pytest.mark.parametrize(
    ("row", "code"),
    [
        ("0 a.map 4 4 0 0 1 1 1.41", "unsupported_scaled_scenario"),
        ("0 a.map 2 2 0 0 2 1 1.41", "out_of_bounds"),
        ("0 ../a.map 2 2 0 0 1 1 1.41", "invalid_map_path"),
        ("0 a.map 2 2 0 0 1 1 nan", "invalid_optimum"),
        ("0 a.map 2 2 0 0 1 1", "invalid_row"),
    ],
)
def test_scenario_rejects_invalid_rows(tmp_path: Path, row: str, code: str) -> None:
    _map(tmp_path, "a.map", ("..", ".."))
    with pytest.raises(DatasetError) as failure:
        parse_scenario(_scenario(tmp_path, [row]), tmp_path)
    assert failure.value.code == code


def test_blocked_endpoint_and_invalid_version(tmp_path: Path) -> None:
    _map(tmp_path, "a.map", ("@.", ".."))
    with pytest.raises(DatasetError, match="blocked endpoint"):
        parse_scenario(_scenario(tmp_path, ["0 a.map 2 2 0 0 1 1 1.41"]), tmp_path)
    with pytest.raises(DatasetError) as failure:
        parse_scenario(
            _scenario(tmp_path, ["0 a.map 2 2 1 0 1 1 1"], version="version 2"), tmp_path
        )
    assert failure.value.code == "invalid_version"


def test_pilot_selection_freezes_distinct_maps_and_query_subset(tmp_path: Path) -> None:
    rows = [f"0 {name} 2 2 0 0 1 1 1.4142" for name in ("a.map", "b.map", "c.map")]
    for name, cells in (("a.map", ("..", "..")), ("b.map", (".@", "..")), ("c.map", (".@", "@."))):
        _map(tmp_path, name, cells)
    # The blocked goal in c.map is still passable; the source row is valid.
    scenarios = parse_scenario(_scenario(tmp_path, rows), tmp_path, family="dao")
    first = select_pilot_cases({"dao": scenarios})
    second = select_pilot_cases({"dao": list(reversed(scenarios))})
    assert [row.map_name for row in first.work_cases] == [row.map_name for row in second.work_cases]
    assert set(row.map_name for row in first.work_cases) == {"a.map", "b.map", "c.map"}
    assert len(first.latency_cases) == 3
    assert first.metadata["work_cases"] == 3
    assert np.count_nonzero(scenarios[2].map.occupancy) == 2
