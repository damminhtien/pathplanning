"""Structural characterization tests use known graphs, never candidate planners."""

from __future__ import annotations

from dataclasses import replace
import json
from pathlib import Path
import zipfile

import numpy as np
import pytest

from scripts.benchmark_datasets.catalog import SCHEMA, TableParser
from scripts.benchmark_datasets.profiles import (
    Limits,
    circle_grid_states,
    hypercube_states,
    load_octile,
    measurement,
    profile_barn,
    profile_continuous,
    profile_dimacs,
    profile_grid,
    profile_scenarios,
    profile_voxel_scenarios,
    profile_voxels,
)
from scripts.benchmark_datasets.report import audit, balanced_cohort, stratify_queries
from scripts.benchmark_datasets.runner import classify, run_profiles, select_entries, unpack


def test_strict_diagonals_and_components_in_2d_and_3d() -> None:
    open_grid = profile_grid(np.zeros((2, 2), dtype=np.bool_), Limits())
    assert open_grid["directed_edges"]["value"] == 12
    diagonal = np.array([[False, True], [True, False]])
    result = profile_grid(diagonal, Limits())
    assert result["directed_edges"]["value"] == 0
    assert result["connected_components"]["value"] == 2
    voxels = np.ones((2, 2, 2), dtype=np.bool_)
    voxels[0, 0, 0] = voxels[1, 1, 1] = False
    result = profile_grid(voxels, Limits())
    assert result["directed_edges"]["value"] == 0
    assert result["connected_components"]["value"] == 2
    assert (
        profile_grid(np.zeros((2, 2, 2), dtype=np.bool_), Limits())["directed_edges"]["value"] == 56
    )


def test_blocked_grid_and_resource_limit_are_not_measured_zero() -> None:
    empty = profile_grid(np.ones((3, 3), dtype=np.bool_), Limits())
    assert empty["free_nodes"]["value"] == 0
    assert empty["largest_component_fraction"]["value"] is None
    limited = profile_grid(np.zeros((3, 3), dtype=np.bool_), Limits(max_cells=8))
    assert limited["free_nodes"]["value"] == 9
    assert limited["directed_edges"] == {
        "value": None,
        "method": "unavailable",
        "reason": "cell_limit",
    }
    with pytest.raises(ValueError, match="reason"):
        measurement(None)


def test_voxel_and_reverse_voxel_equivalence_and_duplicate_accounting(tmp_path: Path) -> None:
    regular = tmp_path / "regular.3dmap"
    reverse = tmp_path / "reverse.3dmap"
    regular.write_text("voxel 2 2 2\n0 0 0\n")
    reverse.write_text(
        "rev_voxel 2 2 2\n"
        + "".join(
            f"{x} {y} {z}\n" for x in range(2) for y in range(2) for z in range(2) if x or y or z
        )
    )
    assert profile_voxels(regular, Limits())[0] == profile_voxels(reverse, Limits())[0]
    regular.write_text("voxel 2 2 2\n0 0 0\n0 0 0\n")
    metrics, _ = profile_voxels(regular, Limits())
    assert metrics["duplicate_coordinate_rows"]["value"] == 1
    assert metrics["free_nodes"]["value"] == 7
    regular.write_text("voxel 1000000 1000000 1000000\n")
    metrics, _ = profile_voxels(regular, Limits())
    assert metrics["node_slots"]["value"] == 10**18
    assert metrics["free_nodes"]["reason"] == "cell_limit"


def test_terrain_symbols_and_cost_claims_do_not_reuse_land_model(tmp_path: Path) -> None:
    terrain = tmp_path / "Map1.map"
    terrain.write_text("type octile\nheight 2\nwidth 2\nmap\nAT\nGW\n")
    blocked, metadata = load_octile(terrain, terrain=True)
    assert not blocked.any()
    assert metadata["terrain_counts"] == {"A": 1, "T": 1, "G": 1, "W": 1}
    with pytest.raises(ValueError, match="Unknown"):
        load_octile(terrain)
    scenario = tmp_path / "Map1.scen"
    scenario.write_text("version 1\n0 Map1.map 2 2 0 0 1 1 2.0\n0 Map1.map 2 2 0 0 1 1 2.0\n")
    metrics = profile_scenarios(scenario, [2, 2], Limits(), terrain=True)
    assert metrics["duplicates"]["value"] == 1
    assert metrics["recorded_optimal_cost"]["reason"] == "source_terrain_cost_table_missing"
    assert metrics["reachability"]["value"] is None


def test_directed_dimacs_preserves_multiarcs_and_distinguishes_scc(tmp_path: Path) -> None:
    path = tmp_path / "tiny.gr"
    path.write_text("c directed fixture\np sp 4 5\na 1 2 1\na 2 1 2\na 2 3 3\na 2 3 4\na 4 4 0\n")
    metrics = profile_dimacs(path, Limits())
    assert metrics["arcs"]["value"] == 5
    assert metrics["weak_components"]["value"] == 2
    assert metrics["strong_components"]["value"] == 3
    assert metrics["parallel_arcs"]["value"] == 1
    assert metrics["self_loops"]["value"] == 1
    assert metrics["zero_cost_arcs"]["value"] == 1
    assert metrics["out_degree"]["value"]["max"] == 3
    path.write_text("p sp 2 1\na 1 3 1\n")
    with pytest.raises(ValueError, match="range"):
        profile_dimacs(path, Limits())


def test_continuous_samples_do_not_prove_connectivity_or_impossibility() -> None:
    limits = Limits(sample_size=1000)
    bounds = np.array([[-5.0, -5.0], [5.0, 5.0]])
    result = profile_continuous(circle_grid_states, bounds, limits)
    assert result == profile_continuous(circle_grid_states, bounds, limits)
    assert result["vertices"]["reason"] == "continuous_space"
    assert result["connected_components"]["value"] is None
    empty_sample = profile_continuous(
        lambda points: np.zeros(len(points), dtype=np.bool_), bounds, limits
    )
    assert empty_sample["free_volume_fraction"]["value"] == 0
    assert empty_sample["free_volume_fraction"]["ci95"][1] > 0


def test_hypercube_predicate_matches_ompl_implementation_not_stale_comment() -> None:
    points = np.array([[0.8, 0.05], [0.05, 0.8], [0.95, 0.8], [0.5, 0.5], [0.0, 0.0], [1.0, 1.0]])
    assert hypercube_states(points).tolist() == [True, False, True, False, True, True]


def test_barn_composes_model_link_and_collision_poses(tmp_path: Path) -> None:
    path = tmp_path / "rotated.world"
    path.write_text(
        "<sdf><world><model><pose>10 20 0 0 0 1.5707963267948966</pose>"
        "<link><pose>1 0 0 0 0 0</pose><collision><pose>1 0 0 0 0 0</pose>"
        "<geometry><cylinder><radius>0.5</radius></cylinder></geometry>"
        "</collision></link></model></world></sdf>"
    )
    metrics, metadata = profile_barn(path, Limits())
    assert np.asarray(metadata["bounds"]) == pytest.approx(np.array([[9.5, 21.5], [10.5, 22.5]]))
    assert metrics["obstacle_count"]["value"] == 1
    assert metadata["source_navigation_scores_comparable"] is False


def test_voxel_queries_keep_source_ratios_and_direction(tmp_path: Path) -> None:
    path = tmp_path / "tiny.3dscen"
    path.write_text("version 2\ntiny.3dmap\n0 0 0 1 1 1 1.732 1.0 2.0\n1 1 1 0 0 0 1.732 1.0 3.0\n")
    metrics = profile_voxel_scenarios(path, [2, 2, 2])
    assert metrics["reverse_pair_count"]["value"] == 2
    assert metrics["normalized_displacement"]["value"]["median"] == 1
    assert metrics["source_search_work_ratio"]["method"] == "source_unverified"
    path.write_text("version 1\ntiny.3dmap\n0 0 0 1 1 1 1.732 1.0\n")
    assert profile_voxel_scenarios(path, [2, 2, 2])["source_search_work_ratio"]["value"] is None


def test_comparison_keys_preserve_cost_motion_and_dimension() -> None:
    land = _entry("land")
    terrain = {**land, "representation": "terrain2d", "metadata": {"cost_model": "unknown"}}
    voxel = {**land, "representation": "voxel3d"}
    keys = {classify(entry, {}, {})["comparison_key"] for entry in (land, terrain, voxel)}
    assert len(keys) == 3
    assert (
        classify(land, {}, {})["comparison_key"]
        == classify(
            land, {}, {"dimensions": [10, 20], "movement_profile": "strict_octile_topology"}
        )["comparison_key"]
    )


def _entry(identifier: str, *, family: str = "maze") -> dict:
    return {
        "dataset_id": identifier,
        "source": "movingai",
        "family": family,
        "name": identifier,
        "url": f"https://example.org/{identifier}.map",
        "representation": "grid2d",
        "metadata": {
            "lineage": f"movingai:{family}:{identifier}",
            "directed": False,
            "cost_model": "euclidean_grid",
        },
    }


def test_selection_is_independent_of_order_and_outcomes() -> None:
    entries = [_entry(str(i)) for i in range(20)]
    first = select_entries(entries, per_family=3, seed=7)
    assert first == select_entries(list(reversed(entries)), per_family=3, seed=7)
    assert len(first) == 3
    assert len(select_entries(entries, per_family=0, seed=7)) == 20


def test_unknown_reachability_and_failed_candidates_are_not_filtered() -> None:
    queries = [
        {
            "workload_id": str(i),
            "map_id": "map",
            "normalized_displacement": i / 10,
            "reference_state": "unknown",
            "candidate_success": i % 2 == 0,
        }
        for i in range(11)
    ]
    selected = stratify_queries(queries, per_bin=20)
    assert len(selected["selected_ids"]) == 11
    assert len(selected["strata"]) == 5
    assert selected == stratify_queries(list(reversed(queries)), per_bin=20)
    with pytest.raises(ValueError, match="Duplicate"):
        stratify_queries(queries + queries[:1])


def test_archive_expansion_limits_and_untrusted_paths(tmp_path: Path) -> None:
    path = tmp_path / "asset.zip"
    with zipfile.ZipFile(path, "w") as archive:
        archive.writestr("../../outside.map", "type octile")
    result = unpack(path, tmp_path, ".map", 1000)
    assert result.parent == tmp_path
    assert result.read_text() == "type octile"
    with pytest.raises(ValueError, match="expansion_limit"):
        unpack(path, tmp_path, ".map", 1)


def test_coverage_and_family_weights_retain_unmeasured_maps() -> None:
    entries = [_entry("a"), _entry("b"), _entry("c", family="room")]
    records = [
        {
            "dataset_id": entry["dataset_id"],
            "status": "not_selected",
            "classification": classify(entry, {}, {}),
        }
        for entry in entries
    ]
    result = {
        "catalog": {"entries": entries, "discovery_failures": [], "scope": {}},
        "records": records,
    }
    summary = audit(result)
    assert summary["families"][0]["catalog_count"] == 2
    assert summary["claims"]["bias_eliminated"] is False
    weights = balanced_cohort(result)["strata"]
    assert sum(row["map_weight"] for row in weights) == pytest.approx(1)
    assert [row["map_weight"] for row in weights] == [0.25, 0.25, 0.5]


def test_discovery_parser_keeps_every_map_row() -> None:
    parser = TableParser()
    parser.feed(
        '<table><tr><td><a href="a.map.zip">a.map</a></td><td>10x20</td></tr>'
        '<tr><td><a href="b.map">b.map</a></td></tr></table>'
    )
    assert len(parser.rows) == 2
    assert parser.rows[0]["links"] == ["a.map.zip"]


def test_original_scaled_and_mirror_entries_share_lineage_weight() -> None:
    original = _entry("original", family="bgmaps")
    scaled = _entry("scaled", family="bg512")
    for entry in (original, scaled):
        entry["metadata"]["lineage"] = "movingai:baldurs_gate:map"
    mirror = _entry("mirror", family="warframe")
    mirror["metadata"]["version"] = "legacy_2018"
    entries = [original, scaled, mirror, _entry("other")]
    result = {
        "catalog": {"entries": entries},
        "records": [
            {"dataset_id": entry["dataset_id"], "classification": classify(entry, {}, {})}
            for entry in entries
        ],
    }
    strata = balanced_cohort(result)["strata"]
    assert len(strata) == 2
    variants = next(row for row in strata if row["family"] == "movingai:baldurs_gate")
    assert variants["dataset_ids"] == ["original", "scaled"]
    assert variants["map_weight"] == 0.5
    assert variants["variant_weight"] == 0.25


def test_movingai_discovers_map_scenario_zip(monkeypatch: pytest.MonkeyPatch) -> None:
    from scripts.benchmark_datasets import catalog

    parser = TableParser()
    parser.feed(
        '<table><tr><td><a href="maze.map.zip">maze.map</a></td><td>512x512</td>'
        '<td><a href="maze.map-scen.zip">maze.scen</a></td></tr></table>'
    )
    monkeypatch.setattr(catalog, "_parse", lambda url: parser)
    entries = catalog.movingai_catalog("maze")
    assert len(entries) == 1
    assert entries[0]["metadata"]["scenario_url"].endswith("/maze.map-scen.zip")


def test_failed_profiles_resume_and_do_not_disappear(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    from scripts.benchmark_datasets import runner

    def fail(*args: object, **kwargs: object) -> None:
        raise ValueError("input failed")

    monkeypatch.setattr(runner, "profile_entry", fail)
    catalog = {"schema_version": SCHEMA, "entries": [_entry("a")], "discovery_failures": []}
    result = run_profiles(catalog, tmp_path, Limits())
    assert result["records"][0]["status"] == "error"
    monkeypatch.setattr(
        runner, "profile_entry", lambda *args, **kwargs: pytest.fail("should resume")
    )
    assert run_profiles(catalog, tmp_path, Limits())["records"] == result["records"]
    assert json.loads((tmp_path / "profiles.json").read_text())["records"] == result["records"]
    catalog["entries"][0]["metadata"]["scenario_url"] = "https://example.org/new.scen"
    monkeypatch.setattr(runner, "profile_entry", fail)
    refreshed = run_profiles(catalog, tmp_path, Limits())
    assert refreshed["config_hash"] != result["config_hash"]
    assert replace(Limits(), seed=8).seed == 8
