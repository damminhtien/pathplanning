"""Oracle checks that do not use the production graph or search routine."""

from __future__ import annotations

import math
from pathlib import Path

import pytest

from scripts.shortest_path_benchmark.reference import (
    ReferenceCache,
    reference_distance,
    scenario_matches_reference,
    validate_path,
)
from scripts.shortest_path_benchmark.workloads import load_map


def _grid(tmp_path: Path, rows: tuple[str, ...]):
    path = tmp_path / "test.map"
    path.write_text(
        f"type octile\nheight {len(rows)}\nwidth {len(rows[0])}\nmap\n" + "\n".join(rows) + "\n"
    )
    return load_map(path)


def test_reference_cost_and_independent_path_checks(tmp_path: Path) -> None:
    grid = _grid(tmp_path, ("..", ".."))
    cache = ReferenceCache()
    result = cache.solve(grid, 0, 3)
    assert result.cost == pytest.approx(math.sqrt(2))
    assert result.path == (0, 3)
    assert cache.solve(grid, 0, 3) is result
    assert reference_distance(grid, 0, 3, cache) == result.cost
    direct = validate_path(grid, [0, 3], 0, 3, declared_cost=math.sqrt(2), oracle_cost=result.cost)
    assert direct.valid and direct.declared_match and direct.optimal_match
    assert direct.steps == 1
    detour = validate_path(grid, [0, 1, 3], 0, 3, declared_cost=2, oracle_cost=result.cost)
    assert detour.valid and detour.declared_match and not detour.optimal_match
    assert validate_path(grid, [0, 3], 0, 3, declared_cost=2).declared_match is False
    assert validate_path(grid, [3, 0], 0, 3).reason == "endpoint_mismatch"


def test_corner_and_unreachable_and_zero_cost(tmp_path: Path) -> None:
    grid = _grid(tmp_path, (".@", ".."))
    assert reference_distance(grid, 0, 3) == pytest.approx(2.0)
    assert validate_path(grid, [0, 3], 0, 3).reason == "corner_cut"
    assert validate_path(grid, [0, 2, 3], 0, 3).cost == 2.0
    same = validate_path(grid, [0], 0, 0, declared_cost=0, oracle_cost=0)
    assert same.valid and same.cost == 0 and same.steps == 0
    assert reference_distance(grid, 0, 0) == 0

    disconnected = _grid(tmp_path, (".@", "@."))
    assert reference_distance(disconnected, 0, 3) is None
    assert validate_path(disconnected, None, 0, 3).reason == "no_path"


def test_scenario_decimal_tolerance(tmp_path: Path) -> None:
    _grid(tmp_path, ("..", ".."))
    assert scenario_matches_reference("1.4142", math.sqrt(2))
    assert scenario_matches_reference("1.414213", math.sqrt(2))
    assert scenario_matches_reference("786.45288543", 786.4528855298877)
    assert not scenario_matches_reference("1.40", math.sqrt(2))
    assert not scenario_matches_reference("786.45298543", 786.4528855298877)
    assert not scenario_matches_reference("1.4142", None)
