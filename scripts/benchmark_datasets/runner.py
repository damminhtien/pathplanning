"""Reproducible dataset characterization and explicit measurement coverage."""

from __future__ import annotations

from collections import defaultdict
import configparser
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import shutil
from typing import Any
import zipfile

import numpy as np

from scripts.benchmark_datasets.catalog import SCHEMA, cached_fetch
from scripts.benchmark_datasets.profiles import (
    Limits,
    circle_grid_states,
    content_hash,
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
    unavailable,
)


def write_json(path: Path, value: Any) -> None:
    """Atomically save JSON, rejecting nonfinite values."""
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_suffix(path.suffix + ".part")
    temp.write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n")
    temp.replace(path)


def unpack(path: Path, cache: Path, suffix: str, maximum_bytes: int) -> Path:
    """Extract exactly one named payload; never trust archive paths or expansion size."""
    if not zipfile.is_zipfile(path):
        return path
    with zipfile.ZipFile(path) as archive:
        candidates = [info for info in archive.infolist() if info.filename.endswith(suffix)]
        if len(candidates) != 1:
            raise ValueError("Expected exactly one matching archive payload")
        item = candidates[0]
        if item.file_size > maximum_bytes:
            raise ValueError("expansion_limit")
        destination = cache / (content_hash(path) + suffix)
        if not destination.exists():
            temp = destination.with_suffix(destination.suffix + ".part")
            with archive.open(item) as source, temp.open("wb") as output:
                shutil.copyfileobj(source, output, length=1 << 20)
            temp.replace(destination)
        return destination


def select_entries(entries: list[dict[str, Any]], *, per_family: int, seed: int) -> set[str]:
    """Freeze a uniform hash sample per source/family before any measurements.

    Zero selects every entry. Sampling does not use planner outcomes, advertised
    optima, filenames, or measured graph properties. Unselected rows are retained.
    """
    if per_family < 0:
        raise ValueError("per_family cannot be negative")
    groups: dict[tuple[str, str], list[dict[str, Any]]] = defaultdict(list)
    for entry in entries:
        groups[(entry["source"], entry["family"])].append(entry)
    selected = set()
    for group in groups.values():
        ordered = sorted(
            group,
            key=lambda item: hashlib.sha256(f"{seed}|{item['dataset_id']}".encode()).hexdigest(),
        )
        selected.update(
            item["dataset_id"] for item in (ordered[:per_family] if per_family else ordered)
        )
    # Generator definitions are a small complete set, not a statistical sample.
    selected.update(
        item["dataset_id"]
        for item in entries
        if item["metadata"].get("format") in {"generator_source", "ompl_cfg"}
    )
    return selected


class LocalDatasetAssets:
    """Resolve and verify installed assets before falling back to network cache."""

    def __init__(self, root: Path, index_path: Path) -> None:
        self.root = root.expanduser().resolve()
        self.index_path = index_path.expanduser().resolve()
        index = json.loads(self.index_path.read_text())
        self.datasets = index["datasets"]
        self.assets = index["assets"]
        self.verified: set[str] = set()

    def fetch(
        self,
        entry: dict[str, Any],
        url: str,
        cache: Path,
        maximum_bytes: int,
    ) -> Path:
        candidates: list[dict[str, Any]] = []
        dataset = self.datasets.get(entry["dataset_id"], {})
        for asset_id in dataset.get("asset_ids", []):
            asset = self.assets.get(asset_id)
            if asset and url in {asset.get("url"), asset.get("download_url")}:
                candidates.append(asset)
        if not candidates:
            candidates = [
                asset
                for asset in self.assets.values()
                if url in {asset.get("url"), asset.get("download_url")}
            ]

        for asset in candidates:
            relative = Path(asset["relative_path"])
            path = (self.root / relative).resolve()
            if not path.is_relative_to(self.root):
                raise ValueError("installed asset path escapes dataset root")
            if not path.is_file():
                continue
            if path.stat().st_size > maximum_bytes:
                raise ValueError("download_limit")
            asset_id = str(asset["asset_id"])
            if asset_id not in self.verified:
                if content_hash(path) != asset["sha256"]:
                    raise ValueError(f"installed asset hash mismatch: {relative}")
                self.verified.add(asset_id)
            return path
        return cached_fetch(url, cache, maximum_bytes=maximum_bytes)


def _cfg(path: Path) -> tuple[dict[str, Any], dict[str, Any]]:
    parser = configparser.ConfigParser(strict=False)
    parser.read(path)
    if "problem" not in parser:
        raise ValueError("OMPL configuration has no problem section")
    problem = dict(parser["problem"])
    control = problem.get("control") or problem.get("control.type")
    state_space = "SE3" if "start.z" in problem else "SE2"
    metrics = {
        "vertices": unavailable("continuous_space"),
        "arcs": unavailable("continuous_space"),
        "topology": unavailable("robot_mesh_collision_checker_required"),
    }
    return metrics, {
        "state_space": state_space,
        "intrinsic_dimension": 6 if state_space == "SE3" else 3,
        "ambient_workspace_dimension": 3 if state_space == "SE3" else 2,
        "robot": problem.get("robot"),
        "world": problem.get("world"),
        "objective": problem.get("objective", "source_default"),
        "control_model": control,
        "source_problem": problem,
        "execution_supported": False,
    }


def classify(
    entry: dict[str, Any], metrics: dict[str, Any], metadata: dict[str, Any]
) -> dict[str, Any]:
    """Use explicit semantics and observable descriptors, not a guessed difficulty score."""
    representation = entry["representation"]
    classes = {
        "grid2d": "regular_2d_uniform_land",
        "terrain2d": "regular_2d_terrain_cost_unknown",
        "voxel3d": "regular_3d_uniform_voxel",
        "directed_graph": "explicit_directed_weighted",
        "continuous2d": "continuous_workspace",
        "configuration_space": "configuration_space",
    }
    dimensions = metadata.get("dimensions", entry["metadata"].get("dimensions_claim"))
    intrinsic = metadata.get("intrinsic_dimension")
    if intrinsic is None and isinstance(dimensions, list):
        intrinsic = len(dimensions)
    if intrinsic is None:
        intrinsic = {"grid2d": 2, "terrain2d": 2, "voxel3d": 3, "continuous2d": 2}.get(
            representation
        )
    vertices = metrics.get("free_nodes", metrics.get("vertices", {})).get("value")
    fraction = metrics.get("obstacle_fraction", {}).get("value")
    semantic_class = classes[representation]
    if metadata.get("state_space") in {"SE2", "SE3", "Rn"}:
        semantic_class = "configuration_space"
    if metadata.get("control_model"):
        semantic_class = "kinodynamic_configuration_space"
    movement = metadata.get("movement_profile") or {
        "grid2d": "strict_octile_topology",
        "terrain2d": "strict_octile_topology",
        "voxel3d": "strict_26_euclidean",
    }.get(representation)
    robot = metadata.get("robot_model", metadata.get("robot"))
    bounds_policy = metadata.get("bounds_policy")
    if entry["source"] == "barn":
        robot, bounds_policy = "point_xy", "obstacle_envelope"
    comparison = {
        "semantic_class": semantic_class,
        "dimension": intrinsic,
        "cost_model": entry["metadata"].get("cost_model", "source_defined"),
        "movement": movement,
        "state_space": metadata.get("state_space"),
        "robot": robot,
        "objective": metadata.get("objective"),
        "control_model": metadata.get("control_model"),
        "bounds_policy": bounds_policy,
    }
    comparison_key = hashlib.sha256(json.dumps(comparison, sort_keys=True).encode()).hexdigest()
    return {
        "semantic_class": semantic_class,
        "comparison_key": comparison_key,
        "comparison_semantics": comparison,
        "intrinsic_dimension": intrinsic,
        "directed": entry["metadata"].get("directed"),
        "cost_model": entry["metadata"].get("cost_model", "source_defined"),
        "size_decade": len(str(vertices)) - 1 if isinstance(vertices, int) and vertices else None,
        "obstacle_density_band": (
            None
            if fraction is None
            else "low"
            if fraction < 0.2
            else "medium"
            if fraction < 0.6
            else "high"
        ),
        "lineage": entry["metadata"].get("lineage"),
        "family_label_is_source_metadata": True,
    }


def profile_entry(
    entry: dict[str, Any],
    cache: Path,
    limits: Limits,
    *,
    maximum_bytes: int,
    local_assets: LocalDatasetAssets | None = None,
) -> dict[str, Any]:
    """Profile one asset and retain its source and decoded hashes."""
    fetch_asset = (
        (lambda url: local_assets.fetch(entry, url, cache, maximum_bytes))
        if local_assets is not None
        else (lambda url: cached_fetch(url, cache, maximum_bytes=maximum_bytes))
    )
    path = fetch_asset(entry["url"])
    raw_hash = content_hash(path)
    representation = entry["representation"]
    source_format = entry["metadata"].get("format")
    suffix = ".3dmap" if representation == "voxel3d" else ".map"
    path = unpack(path, cache, suffix, maximum_bytes)
    metadata: dict[str, Any] = {}
    queries: dict[str, Any] = {}
    if source_format == "generator_source":
        if entry["name"] == "circle_grid":
            metrics = profile_continuous(
                circle_grid_states, np.array([[-5.0, -5.0], [5.0, 5.0]]), limits
            )
            metadata = {
                "state_space": "SE2",
                "intrinsic_dimension": 3,
                "radius": 0.25,
                "bounds_xy": [[-5.0, -5.0], [5.0, 5.0]],
                "volume_measure": "XY_marginal_orientation_independent",
                "source_default_variant": True,
            }
            metrics["analytic_free_volume_fraction"] = measurement(1 - np.pi * 0.25**2, "analytic")
        elif entry["name"] == "hypercube":
            dimensions = (2, 3, 6, 10)
            metrics = {
                "dimension_sweep": measurement(
                    {
                        str(dimension): profile_continuous(
                            hypercube_states,
                            np.array([[0.0] * dimension, [1.0] * dimension]),
                            limits,
                        )
                        for dimension in dimensions
                    },
                    "uniform_sample",
                )
            }
            metadata = {
                "state_space": "Rn",
                "intrinsic_dimension": 6,
                "source_default_dimension": 6,
                "dimension_sweep": list(dimensions),
                "edge_width": 0.1,
                "corridor_order": "prefix_high_suffix_low_as_source_code",
            }
            metrics["analytic_free_volume_by_dimension"] = measurement(
                {
                    str(dimension): dimension * 0.1 ** (dimension - 1)
                    - (dimension - 1) * 0.1**dimension
                    for dimension in dimensions
                },
                "analytic",
            )
        else:
            metrics = {"topology": unavailable("generator_requires_specialized_space_adapter")}
            metadata = {"execution_supported": False}
    elif source_format == "ompl_cfg":
        metrics, metadata = _cfg(path)
    elif representation == "continuous2d":
        metrics, metadata = profile_barn(path, limits)
        metadata["intrinsic_dimension"] = 2
        path_url = (
            entry["metadata"].get("path_url")
            or entry["url"].replace("/world_", "/path_files/path_").removesuffix(".world") + ".npy"
        )
        if path_url:
            source_path = np.load(fetch_asset(path_url), allow_pickle=False)
            if (
                source_path.ndim != 2
                or source_path.shape[1] != 2
                or source_path.shape[0] < 2
                or not np.all(np.isfinite(source_path))
            ):
                raise ValueError("Invalid BARN supplied path")
            steps = np.linalg.norm(np.diff(source_path, axis=0), axis=1)
            chord = float(np.linalg.norm(source_path[-1] - source_path[0]))
            queries = {
                "supplied_path_length_cells": measurement(float(steps.sum()), "source_path"),
                "supplied_path_tortuosity": (
                    measurement(float(steps.sum()) / chord, "source_path")
                    if chord
                    else unavailable("coincident_endpoints")
                ),
                "optimality": unavailable("supplied_path_not_an_optimality_proof"),
            }
            metadata["supplied_path_coordinates"] = "row_column_cells_not_world_xy"
    elif representation == "directed_graph":
        metrics = profile_dimacs(path, limits)
    elif representation == "voxel3d":
        metrics, metadata = profile_voxels(path, limits)
        metadata["movement_profile"] = "strict_26_euclidean"
        scenario_url = entry["metadata"].get("scenario_url")
        if scenario_url:
            scenario = unpack(
                fetch_asset(scenario_url),
                cache,
                ".3dscen",
                maximum_bytes,
            )
            queries = profile_voxel_scenarios(scenario, metadata["dimensions"])
            metadata["scenario_sha256"] = content_hash(scenario)
    else:
        terrain = representation == "terrain2d"
        blocked, metadata = load_octile(path, terrain=terrain)
        metrics = profile_grid(blocked, limits)
        metadata["movement_profile"] = "strict_octile_topology"
        if terrain:
            metrics["edge_cost"] = unavailable("source_terrain_cost_table_missing")
        scenario_url = entry["metadata"].get("scenario_url")
        if scenario_url:
            scenario = unpack(
                fetch_asset(scenario_url),
                cache,
                ".scen",
                maximum_bytes,
            )
            queries = profile_scenarios(scenario, metadata["dimensions"], limits, terrain=terrain)
            metadata["scenario_sha256"] = content_hash(scenario)
    return {
        "dataset_id": entry["dataset_id"],
        "status": "profiled",
        "metrics": metrics,
        "query_metrics": queries,
        "metadata": metadata,
        "classification": classify(entry, metrics, metadata),
        "asset_sha256": raw_hash,
        "decoded_sha256": content_hash(path),
    }


def run_profiles(
    catalog: dict[str, Any],
    output: Path,
    limits: Limits,
    *,
    per_family: int = 3,
    maximum_bytes: int = 256 << 20,
    cache: Path | None = None,
    dataset_root: Path | None = None,
    dataset_index: Path | None = None,
) -> dict[str, Any]:
    """Resume complete records; account for every catalog entry including errors."""
    if catalog.get("schema_version") != SCHEMA:
        raise ValueError("Unsupported catalog schema")
    selected = select_entries(catalog["entries"], per_family=per_family, seed=limits.seed)
    cache = cache or output / "cache"
    if (dataset_root is None) != (dataset_index is None):
        raise ValueError("dataset_root and dataset_index must be provided together")
    local_assets = (
        LocalDatasetAssets(dataset_root, dataset_index)
        if dataset_root is not None and dataset_index is not None
        else None
    )
    profiler_fingerprint = hashlib.sha256(
        Path(__file__).read_bytes() + Path(__file__).with_name("profiles.py").read_bytes()
    ).hexdigest()
    settings = {
        "limits": asdict(limits),
        "per_family": per_family,
        "maximum_bytes": maximum_bytes,
        "profiler_fingerprint": profiler_fingerprint,
        "catalog_fingerprint": hashlib.sha256(
            json.dumps(catalog["entries"], sort_keys=True).encode()
        ).hexdigest(),
        "local_dataset_root": str(local_assets.root) if local_assets else None,
        "local_asset_index_sha256": (
            content_hash(local_assets.index_path) if local_assets else None
        ),
    }
    config_hash = hashlib.sha256(json.dumps(settings, sort_keys=True).encode()).hexdigest()
    output.mkdir(parents=True, exist_ok=True)
    records_dir = output / "records" / config_hash
    records_dir.mkdir(parents=True, exist_ok=True)
    records = []
    for entry in catalog["entries"]:
        identifier = entry["dataset_id"]
        saved = records_dir / f"{identifier}.json"
        if identifier not in selected:
            record = {
                "dataset_id": identifier,
                "status": "not_selected",
                "classification": classify(entry, {}, {}),
            }
        elif saved.exists():
            record = json.loads(saved.read_text())
        else:
            print(f"profiling {entry['source']}/{entry['family']}/{entry['name']}", flush=True)
            try:
                record = profile_entry(
                    entry,
                    cache,
                    limits,
                    maximum_bytes=maximum_bytes,
                    local_assets=local_assets,
                )
            except Exception as exc:
                record = {
                    "dataset_id": identifier,
                    "status": (
                        "resource_limited"
                        if isinstance(exc, ValueError)
                        and str(exc) in {"download_limit", "expansion_limit"}
                        else "error"
                    ),
                    "error": f"{type(exc).__name__}: {exc}",
                    "classification": classify(entry, {}, {}),
                }
            write_json(saved, record)
        records.append(record)
    result = {
        "schema_version": SCHEMA,
        "settings": settings,
        "config_hash": config_hash,
        "catalog": catalog,
        "records": records,
        "selection": {
            "method": "uniform_hash_per_source_family",
            "seed": limits.seed,
            "selected_ids": sorted(selected),
        },
    }
    write_json(output / "profiles.json", result)
    return result
