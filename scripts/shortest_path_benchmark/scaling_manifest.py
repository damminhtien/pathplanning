"""Freeze deterministic generated-grid scaling workloads for the native runner."""

from __future__ import annotations

from collections import defaultdict
import hashlib
import json
from pathlib import Path
from typing import Any, Iterable

from scripts.shortest_path_benchmark.contract import canonical_json, stable_id
from scripts.shortest_path_benchmark.runner import (
    SCHEMA,
    VARIANTS,
    _variant_identity_parameters,
    _write_json,
)
from scripts.shortest_path_benchmark.scaling import (
    ScalingCase,
    density_sweep,
    size_sweep,
    topology_sweep,
)
from scripts.shortest_path_benchmark.workloads import MOVEMENT_PROFILE


def _variants(alphas: Iterable[float]) -> list[dict[str, Any]]:
    variants = []
    for variant in VARIANTS:
        variant_name = variant["id"]
        identity = {
            "algorithm": variant["planner"],
            "parameters": _variant_identity_parameters(variant),
            "heuristic_mode": "octile_precomputed_array" if variant["weight"] else "zero",
            "tie_policy": "f_ascending_h_ascending_insertion_ascending",
        }
        manifest_variant = {
            **variant,
            "variant_id": stable_id("variant", identity),
            "variant_name": variant_name,
        }
        if "anytime_weights" in manifest_variant:
            manifest_variant["anytime_weights"] = list(manifest_variant["anytime_weights"])
        variants.append(manifest_variant)
    for alpha in alphas:
        variant_name = f"astar_halpha_{alpha:g}"
        identity = {
            "algorithm": "astar",
            "parameters": {"heuristic_alpha": alpha},
            "heuristic_mode": "scaled_octile_precomputed_array",
            "tie_policy": "f_ascending_h_ascending_insertion_ascending",
        }
        variants.append(
            {
                "id": variant_name,
                "planner": "astar",
                "algorithm": 4,
                "weight": 1.0,
                "heuristic_alpha": alpha,
                "variant_id": stable_id("variant", identity),
                "variant_name": variant_name,
            }
        )
    return variants


def _write_map(path: Path, blocked: Any) -> str:
    height, width = blocked.shape
    rows = ["".join("@" if value else "." for value in row) for row in blocked]
    payload = f"type octile\nheight {height}\nwidth {width}\nmap\n" + "\n".join(rows) + "\n"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(payload, encoding="ascii")
    return hashlib.sha256(payload.encode("ascii")).hexdigest()


def _sweep_cases(profile: dict[str, Any]) -> Iterable[ScalingCase]:
    size = profile["size_sweep"]
    density = profile["density_sweep"]
    queries_per_seed = int(size.get("queries_per_seed", 20))
    if queries_per_seed < 5 or queries_per_seed % 5:
        raise ValueError("queries_per_seed must be a positive multiple of five")
    queries_per_bin = queries_per_seed // 5
    yield from size_sweep(
        sizes=tuple(size["side_lengths"]),
        seeds=tuple(size["seeds"]),
        density=float(size["obstacle_density"]),
        queries_per_bin=queries_per_bin,
    )
    yield from density_sweep(
        size=int(density["side_length"]),
        densities=tuple(float(value) for value in density["densities"]),
        seeds=tuple(density["seeds"]),
        queries_per_bin=queries_per_bin,
    )
    if profile.get("corridor_widths"):
        yield from topology_sweep(
            size=int(profile.get("topology_side_length", density["side_length"])),
            opening_widths=tuple(int(value) for value in profile["corridor_widths"]),
            seeds=tuple(size["seeds"]),
            queries_per_bin=queries_per_bin,
        )


def prepare_scaling_manifest(profile_path: str | Path, output_path: str | Path) -> dict[str, Any]:
    """Write maps and a hash-pinned workload manifest before any planner runs."""
    profile_file = Path(profile_path).expanduser().resolve()
    output = Path(output_path).expanduser().resolve()
    profile = json.loads(profile_file.read_text())
    if profile.get("generator_version") != "grid_sweep_v1":
        raise ValueError("unsupported scaling generator version")
    dataset_root = output.parent
    maps_dir = dataset_root / "inputs"
    map_assets: dict[str, dict[str, str]] = {}
    cases_by_id: dict[str, dict[str, Any]] = {}
    workload_bins: dict[str, int] = {}
    map_query_ids: dict[str, list[str]] = {}
    generated_query_count = 0

    for scale_case in _sweep_cases(profile):
        case_descriptor = scale_case.manifest_record()
        case_id = str(case_descriptor["case_id"])
        map_relative = f"inputs/{case_id}.map"
        map_hash = _write_map(maps_dir / f"{case_id}.map", scale_case.blocked)
        map_assets[map_relative] = {"path": map_relative, "sha256": map_hash}
        map_query_ids[map_relative] = []
        diagonal = max(1.0, (2 * (scale_case.size - 1) ** 2) ** 0.5)
        for query in scale_case.queries:
            generated_query_count += 1
            start_x, start_y = query.start
            goal_x, goal_y = query.goal
            start = start_y * scale_case.size + start_x
            goal = goal_y * scale_case.size + goal_x
            identity = {
                "map_sha256": map_hash,
                "movement_profile": MOVEMENT_PROFILE,
                "start": start,
                "goal": goal,
            }
            workload_id = stable_id("workload", identity)
            family = f"scaling:{scale_case.sweep}:{scale_case.topology}"
            row = cases_by_id.get(workload_id)
            if row is None:
                row = {
                    "workload_id": workload_id,
                    "family": family,
                    "map_path": map_relative,
                    "map_sha256": map_hash,
                    "map_occupancy_sha256": case_descriptor["map_sha256"],
                    "scenario_path": None,
                    "scenario_line": None,
                    "bucket": query.displacement_bin,
                    "start": start,
                    "goal": goal,
                    "scenario_optimum": None,
                    "reference_cost": None,
                    "reference_reachable": None,
                    "reference_expanded": None,
                    "node_slots": scale_case.size * scale_case.size,
                    "free_nodes": int((~scale_case.blocked).sum()),
                    "directed_edges": None,
                    "movement_profile": MOVEMENT_PROFILE,
                    "query_kind": "synthetic_grid",
                    "cohort": "work",
                    "sweep": scale_case.sweep,
                    "topology": scale_case.topology,
                    "source_seed": scale_case.seed,
                    "requested_density": scale_case.requested_density,
                    "observed_density": case_descriptor["observed_density"],
                    "opening_width": scale_case.opening_width,
                    "connected_components_4": scale_case.connected_components,
                    "largest_component_nodes_4": scale_case.largest_component_nodes,
                    "displacement_bin": query.displacement_bin,
                    "normalized_displacement": (
                        ((goal_x - start_x) ** 2 + (goal_y - start_y) ** 2) ** 0.5
                        / diagonal
                    ),
                    "sweep_memberships": [],
                }
                cases_by_id[workload_id] = row
                workload_bins[workload_id] = query.displacement_bin
            if family not in row["sweep_memberships"]:
                row["sweep_memberships"].append(family)
            map_query_ids[map_relative].append(workload_id)

    cases = sorted(cases_by_id.values(), key=lambda row: row["workload_id"])
    latency_ids = []
    memory_ids = []
    for workload_ids in map_query_ids.values():
        grouped: dict[int, list[str]] = {}
        for workload_id in workload_ids:
            grouped.setdefault(workload_bins[workload_id], []).append(workload_id)
        for bin_ids in grouped.values():
            selected = min(
                set(bin_ids),
                key=lambda workload_id: hashlib.sha256(
                    f"{profile.get('seed', 7)}|{workload_id}".encode()
                ).hexdigest(),
            )
            latency_ids.append(selected)
        if workload_ids:
            memory_ids.append(
                min(
                    set(workload_ids),
                    key=lambda workload_id: hashlib.sha256(
                        f"{profile.get('seed', 7)}|memory|{workload_id}".encode()
                    ).hexdigest(),
                )
            )

    variants = _variants(tuple(float(value) for value in profile.get("heuristic_alpha", ())))
    sweep_workloads: dict[str, set[str]] = defaultdict(set)
    for row in cases:
        for membership in row["sweep_memberships"]:
            _, sweep_name, _ = membership.split(":", 2)
            sweep_workloads[sweep_name].add(row["workload_id"])
    manifest: dict[str, Any] = {
        "schema_version": SCHEMA,
        "profile": "scaling",
        "movement_profile": MOVEMENT_PROFILE,
        "dataset_root": str(dataset_root),
        "workload_seed": int(profile.get("seed", 7)),
        "cases": cases,
        "selection": {
            "generator_version": profile["generator_version"],
            "raw_generated_queries": generated_query_count,
            "unique_workloads": len(cases),
            "map_count": len(map_assets),
            "latency_queries_per_map": "one hash-selected query per displacement bin",
            "workload_bins": workload_bins,
            "profile": profile,
        },
        "dataset_attribution": {
            "title": "Pathplanning deterministic generated-grid scaling profile",
            "source": "scripts/shortest_path_benchmark/scaling.py",
            "generator_version": profile["generator_version"],
        },
        "cohorts": {
            "work": {"workload_ids": [row["workload_id"] for row in cases]},
            "latency": {"workload_ids": sorted(set(latency_ids))},
            "memory": {
                "workload_ids": sorted(set(memory_ids)),
                "selection_policy": "one_hash_selected_query_per_map_v1",
            },
            **{
                sweep_name: {"workload_ids": sorted(workload_ids)}
                for sweep_name, workload_ids in sorted(sweep_workloads.items())
            },
        },
        "variants": variants,
        "exclusions": [],
        "dataset_files": sorted(map_assets.values(), key=lambda row: row["path"]),
    }
    manifest["manifest_hash"] = hashlib.sha256(canonical_json(manifest)).hexdigest()
    _write_json(output, manifest)
    return manifest


__all__ = ["prepare_scaling_manifest"]
