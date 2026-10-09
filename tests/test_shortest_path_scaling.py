from __future__ import annotations

import numpy as np
import pytest

from scripts.shortest_path_benchmark.scaling import (
    ablation_factor,
    connectivity_summary,
    fit_loglog_slope,
    heuristic_variants,
    sample_queries,
)
from scripts.shortest_path_benchmark.scaling_manifest import prepare_scaling_manifest


def test_connectivity_and_sampled_queries_are_reproducible() -> None:
    blocked = np.zeros((32, 32), dtype=np.bool_)
    blocked[:, 16] = True
    components, largest = connectivity_summary(blocked)
    assert (components, largest) == (2, 512)
    first = sample_queries(blocked, seed=7, per_bin=1)
    second = sample_queries(blocked, seed=7, per_bin=1)
    assert first == second
    assert {query.displacement_bin for query in first} == set(range(5))


def test_consistent_heuristic_ablations_and_slope_ci() -> None:
    variants = heuristic_variants()
    assert [variant["alpha"] for variant in variants] == [0, 0.25, 0.5, 0.75, 1]
    estimate = fit_loglog_slope([(10, 100), (20, 400), (40, 1600)], draws=100, seed=7)
    assert estimate["slope"] == pytest.approx(2.0)
    assert (
        estimate["ci95"]
        == fit_loglog_slope([(10, 100), (20, 400), (40, 1600)], draws=100, seed=7)["ci95"]
    )
    assert estimate["interpretation"] == "empirical_trend_only"
    assert (
        ablation_factor({"h": 1, "weight": 1}, {"h": 0.5, "weight": 1}, factor="h")["factor"] == "h"
    )
    with pytest.raises(ValueError, match="only h"):
        ablation_factor({"h": 1, "weight": 1}, {"h": 0.5, "weight": 2}, factor="h")


def test_scaling_manifest_includes_size_density_topology_and_heuristic_sweeps(
    tmp_path,
) -> None:
    import json

    profile = {
        "profile": "scaling-test",
        "generator_version": "grid_sweep_v1",
        "size_sweep": {
            "side_lengths": [32],
            "obstacle_density": 0.2,
            "seeds": [1],
            "queries_per_seed": 20,
        },
        "density_sweep": {"side_length": 32, "densities": [0.2], "seeds": [1]},
        "corridor_widths": [1, 2],
        "heuristic_alpha": [0, 1],
    }
    profile_path = tmp_path / "profile.json"
    profile_path.write_text(json.dumps(profile))
    manifest_path = tmp_path / "scaling" / "manifest.json"

    manifest = prepare_scaling_manifest(profile_path, manifest_path)
    assert manifest["selection"]["map_count"] == 6
    assert manifest["selection"]["raw_generated_queries"] == 120
    assert len(manifest["cohorts"]["work"]["workload_ids"]) == 100
    assert set(manifest["cohorts"]) >= {"size", "density", "topology"}
    assert len(manifest["cohorts"]["size"]["workload_ids"]) == 20
    assert len(manifest["cohorts"]["density"]["workload_ids"]) == 20
    assert len(manifest["cohorts"]["topology"]["workload_ids"]) == 80
    assert len(manifest["cohorts"]["memory"]["workload_ids"]) == 5
    assert {row["topology"] for row in manifest["cases"] if row["sweep"] == "topology"} == {
        "room",
        "serpentine_maze",
    }
    assert any(
        set(row["sweep_memberships"]) == {"scaling:size:random", "scaling:density:random"}
        for row in manifest["cases"]
    )
    assert {row["variant_name"] for row in manifest["variants"] if "halpha" in row["id"]} == {
        "astar_halpha_0",
        "astar_halpha_1",
    }
    first_hash = manifest["manifest_hash"]
    repeated = prepare_scaling_manifest(profile_path, manifest_path)
    assert repeated["manifest_hash"] == first_hash
