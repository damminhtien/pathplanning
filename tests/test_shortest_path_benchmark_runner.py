"""Protocol invariants for the resumable shortest-path runner."""

from __future__ import annotations

import json

from scripts.shortest_path_benchmark.runner import (
    SCHEMA,
    _outcome,
    validate_manifest,
)
from scripts.shortest_path_benchmark.workloads import load_map


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
