"""Reproducible, process-isolated MovingAI shortest-path experiments.

The latency pass always calls the production library. Instrumented work and
process RSS are collected in separate passes so they cannot change that timer.
"""

from __future__ import annotations

from collections import defaultdict
from concurrent.futures import ProcessPoolExecutor
import ctypes
import hashlib
import json
import multiprocessing as mp
import os
from pathlib import Path
import platform
import random
import resource
import shutil
import time
import traceback
from typing import Any

import numpy as np

from pathplanning.api import plan_discrete
from pathplanning.core.contracts import DiscreteProblem
from pathplanning.native._ffi import (
    GraphCsrView,
    GraphStorageInfo,
    SearchOptions,
    SearchResult,
    load_native_library,
)
from scripts.shortest_path_benchmark.contract import (
    append_jsonl,
    canonical_json,
    read_jsonl,
    recover_jsonl_tail,
    stable_id,
    validate_observation,
    write_json_atomic,
)
from scripts.shortest_path_benchmark.reference import (
    ReferenceCache,
    scenario_matches_reference,
    validate_path,
)
from scripts.shortest_path_benchmark.workloads import (
    load_map,
    make_case_record,
    parse_scenario,
    select_pilot_cases,
)

SCHEMA = "pathplanning_shortest_path_v2"
VARIANTS = (
    {"id": "dijkstra", "planner": "dijkstra", "algorithm": 5, "weight": 0.0},
    {"id": "astar", "planner": "astar", "algorithm": 4, "weight": 1.0},
    {
        "id": "bidirectional_dijkstra",
        "planner": "bidirectional_dijkstra",
        "algorithm": 7,
        "weight": 0.0,
    },
    {"id": "bidirectional_astar", "planner": "bidirectional_astar", "algorithm": 9, "weight": 1.0},
    {"id": "weighted_astar_1.25", "planner": "weighted_astar", "algorithm": 6, "weight": 1.25},
    {"id": "weighted_astar_1.5", "planner": "weighted_astar", "algorithm": 6, "weight": 1.5},
    {"id": "weighted_astar_2", "planner": "weighted_astar", "algorithm": 6, "weight": 2.0},
    {"id": "greedy_best_first", "planner": "greedy_best_first", "algorithm": 3, "weight": 1.0},
)


def _canonical(value: Any) -> bytes:
    return canonical_json(value)


def _sha(value: Any) -> str:
    return hashlib.sha256(_canonical(value)).hexdigest()


def _file_sha(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _scenario_map_paths(path: Path, root: Path) -> set[Path]:
    """Scan scenario map names without materializing one object per query."""
    result: set[Path] = set()
    resolved_names: dict[str, Path] = {}
    with path.open(encoding="ascii") as source:
        header = source.readline().strip()
        if header not in {"version 1", "version 1.0"}:
            raise ValueError(f"unsupported scenario header: {path}")
        for line_number, line in enumerate(source, start=2):
            if not line.strip():
                continue
            fields = line.split()
            if len(fields) != 9:
                raise ValueError(f"scenario line {line_number} must have nine fields: {path}")
            name = fields[1]
            map_path = resolved_names.get(name)
            if map_path is None:
                relative = Path(name)
                if relative.is_absolute() or ".." in relative.parts or "\\" in name:
                    raise ValueError(f"scenario map path escapes dataset root: {name!r}")
                candidates = ((root / relative).resolve(), (path.parent / relative).resolve())
                matches = {candidate for candidate in candidates if candidate.is_file()}
                if len(matches) != 1:
                    raise ValueError(f"scenario map path is missing or ambiguous: {name!r}")
                map_path = matches.pop()
                if not map_path.is_relative_to(root):
                    raise ValueError(f"scenario map path escapes dataset root: {name!r}")
                resolved_names[name] = map_path
            result.add(map_path)
    return result


def _rank_maps(
    root: Path, scenarios: list[Path]
) -> tuple[dict[str, set[Path]], dict[str, Any], list[dict[str, str]]]:
    """Rank maps by free cells, loading each map only long enough to count it."""
    scenario_maps: dict[str, set[Path]] = {}
    family_maps: dict[str, set[Path]] = defaultdict(set)
    failures: list[dict[str, str]] = []
    for scenario_path in scenarios:
        family = scenario_path.relative_to(root).parts[0]
        try:
            paths = _scenario_map_paths(scenario_path, root)
            scenario_maps[str(scenario_path)] = paths
            family_maps[family].update(paths)
        except Exception as exc:
            scenario_maps[str(scenario_path)] = set()
            failures.append({"path": str(scenario_path.relative_to(root)), "reason": str(exc)})

    inventory: dict[str, Any] = {}
    for family, maps in sorted(family_maps.items()):
        ranked = []
        for map_path in maps:
            try:
                grid = load_map(map_path)
                ranked.append(
                    {
                        "path": map_path,
                        "relative_path": map_path.relative_to(root).as_posix(),
                        "free_nodes": grid.free_count,
                        "map_sha256": grid.map_sha256,
                    }
                )
            except Exception as exc:
                failures.append({"path": map_path.relative_to(root).as_posix(), "reason": str(exc)})
        inventory[family] = sorted(
            ranked, key=lambda item: (item["free_nodes"], item["relative_path"])
        )
    return scenario_maps, inventory, failures


def _write_json(path: Path, value: Any) -> None:
    write_json_atomic(path, value)


def prepare_manifest(
    dataset_root: str | Path,
    output: str | Path,
    *,
    profile: str = "pilot",
    seed: int = 7,
) -> dict[str, Any]:
    """Freeze input hashes and deterministic query selection before any run."""
    root = Path(dataset_root).expanduser().resolve()
    if not root.is_dir():
        raise ValueError(f"dataset root is not a directory: {root}")
    scenarios = sorted(root.rglob("*.scen"))
    if not scenarios:
        raise ValueError(f"no .scen files under {root}")
    scenarios_by_family: dict[str, list[Any]] = defaultdict(list)
    exclusions: list[dict[str, str]] = []
    if profile == "pilot":
        scenario_maps, inventory, scan_failures = _rank_maps(root, scenarios)
        exclusions.extend(scan_failures)
        selected_paths: dict[str, set[Path]] = {}
        for family, ranked in inventory.items():
            if not ranked:
                continue
            indices = (0, (len(ranked) - 1) // 2, len(ranked) - 1)
            selected_paths[family] = {ranked[index]["path"] for index in indices}
        for scenario_path in scenarios:
            family = scenario_path.relative_to(root).parts[0]
            if not (
                scenario_maps.get(str(scenario_path), set()) & selected_paths.get(family, set())
            ):
                if not scenario_maps.get(str(scenario_path), set()):
                    exclusions.append(
                        {
                            "path": str(scenario_path.relative_to(root)),
                            "reason": "invalid_scenario_or_map_reference",
                        }
                    )
                continue
            try:
                parsed = parse_scenario(scenario_path, root, family=family)
                scenarios_by_family[family].extend(
                    row for row in parsed if row.map_path in selected_paths[family]
                )
            except Exception as exc:
                exclusions.append(
                    {"path": str(scenario_path.relative_to(root)), "reason": str(exc)}
                )
        if not any(scenarios_by_family.values()):
            raise ValueError("no valid selected MovingAI scenarios")
        selection = select_pilot_cases(scenarios_by_family, seed=seed)
        work_scenarios = selection.work_cases
        latency_scenarios = selection.latency_cases
        selection_metadata = selection.metadata
        for family, ranked in inventory.items():
            family_metadata = selection_metadata.get("families", {}).get(family, {})
            family_metadata["available_maps"] = len(ranked)
            family_metadata["map_inventory"] = [
                {key: value for key, value in item.items() if key != "path"} for item in ranked
            ]
            family_metadata["selected_map_paths"] = [
                Path(path).relative_to(root).as_posix()
                for path in sorted(selected_paths.get(family, set()))
            ]
    elif profile == "full":
        for scenario_path in scenarios:
            family = scenario_path.relative_to(root).parts[0]
            try:
                parsed = parse_scenario(scenario_path, root, family=family)
                scenarios_by_family[family].extend(parsed)
            except Exception as exc:
                exclusions.append(
                    {"path": str(scenario_path.relative_to(root)), "reason": str(exc)}
                )
        all_scenarios = [
            scenario for entries in scenarios_by_family.values() for scenario in entries
        ]
        if not all_scenarios:
            raise ValueError("no valid MovingAI scenario rows")
        work_scenarios = tuple(
            sorted(
                all_scenarios,
                key=lambda item: (item.family, item.scenario_path.as_posix(), item.line_number),
            )
        )
        latency_scenarios = work_scenarios
        selection_metadata = {"selection_version": "all_valid_scenarios_v1"}
    else:
        raise ValueError(f"unsupported profile: {profile}")
    selected_by_id = {
        case["workload_id"]: case
        for case in (make_case_record(scenario, None) for scenario in work_scenarios)
    }
    if profile == "pilot":
        bins = selection_metadata["workload_bins"]
        for workload_id, case in selected_by_id.items():
            case["difficulty_bin"] = bins[workload_id]
    selected = sorted(selected_by_id.values(), key=lambda case: case["workload_id"])
    latency_ids = sorted(
        {make_case_record(scenario, None)["workload_id"] for scenario in latency_scenarios}
    )
    variants = []
    for variant in VARIANTS:
        variant_name = variant["id"]
        variant_id = stable_id(
            "variant",
            {
                "algorithm": variant["planner"],
                "parameters": {"weight": variant["weight"]},
                "heuristic_mode": "octile_precomputed_array" if variant["weight"] else "zero",
                "tie_policy": "f_ascending_h_ascending_insertion_ascending",
            },
        )
        variants.append({**variant, "variant_id": variant_id, "variant_name": variant_name})
    manifest = {
        "schema_version": SCHEMA,
        "profile": profile,
        "movement_profile": "land_octile_v1",
        "dataset_root": str(root),
        "workload_seed": seed,
        "cases": selected,
        "selection": selection_metadata,
        "dataset_attribution": {
            "title": "Moving AI 2D Pathfinding Benchmarks, Version 2",
            "url": "https://movingai.com/benchmarks/grids.html",
            "format_url": "https://movingai.com/benchmarks/formats.html",
            "license_url": "https://opendatacommons.org/licenses/odbl/",
        },
        "cohorts": {
            "work": {"workload_ids": [case["workload_id"] for case in selected]},
            "latency": {"workload_ids": latency_ids},
            "memory": {"workload_ids": latency_ids},
        },
        "variants": variants,
        "exclusions": exclusions,
        "dataset_files": sorted(
            [{"path": str(path.relative_to(root)), "sha256": _file_sha(path)} for path in scenarios]
            + [
                {"path": str(path.relative_to(root)), "sha256": _file_sha(path)}
                for path in sorted(root.rglob("*.map"))
            ],
            key=lambda item: item["path"],
        ),
    }
    manifest["manifest_hash"] = _sha({k: v for k, v in manifest.items() if k != "manifest_hash"})
    _write_json(Path(output), manifest)
    return manifest


def load_manifest(path: str | Path) -> dict[str, Any]:
    manifest = json.loads(Path(path).read_text())
    digest = manifest.pop("manifest_hash")
    if _sha(manifest) != digest:
        raise ValueError("manifest hash mismatch")
    manifest["manifest_hash"] = digest
    if manifest.get("schema_version") != SCHEMA:
        raise ValueError("unsupported manifest schema")
    return manifest


def _case_grid(manifest: dict[str, Any], case: dict[str, Any]) -> Any:
    path = (Path(manifest["dataset_root"]) / case["map_path"]).resolve()
    if not path.is_relative_to(Path(manifest["dataset_root"]).resolve()):
        raise ValueError("map path escapes dataset root")
    if _file_sha(path) != case["map_sha256"]:
        raise ValueError(f"map changed since manifest: {path}")
    return load_map(path)


def _solve_oracle_batch(
    job: tuple[str, str, list[dict[str, Any]]],
) -> list[dict[str, Any]]:
    root_path, map_path, cases = job
    grid = load_map(Path(root_path) / map_path)
    cache = ReferenceCache()
    records = []
    for case in cases:
        solved = cache.solve(grid, int(case["start"]), int(case["goal"]))
        records.append(
            {
                "schema_version": SCHEMA,
                "workload_id": case["workload_id"],
                "input": _input(case),
                "outcome": {
                    "reference_cost": float(solved.cost) if solved.reachable else None,
                    "reference_reachable": solved.reachable,
                    "reference_expanded": solved.expanded,
                    "reference_version": "occupancy_dijkstra_v1",
                },
            }
        )
    return records


def validate_manifest(path: str | Path) -> dict[str, Any]:
    """Check source files and independent reference costs before measurement."""
    manifest_path = Path(path)
    manifest = load_manifest(manifest_path)
    discrepancies: list[dict[str, Any]] = []
    oracle_path = manifest_path.with_suffix(".oracle.jsonl")
    recover_jsonl_tail(oracle_path)
    oracle_records = read_jsonl(oracle_path)
    oracle_by_id = {row["workload_id"]: row for row in oracle_records}
    root = Path(manifest["dataset_root"])
    source_files_valid = True
    for source in manifest["dataset_files"]:
        file_path = (root / source["path"]).resolve()
        if (
            not file_path.is_relative_to(root.resolve())
            or not file_path.is_file()
            or _file_sha(file_path) != source["sha256"]
        ):
            discrepancies.append({"path": source["path"], "reason": "source_file_changed"})
            source_files_valid = False
    prior_validation = manifest.get("validation", {})
    reuse_manifest_references = (
        source_files_valid
        and prior_validation.get("checked") == len(manifest["cases"])
        and not prior_validation.get("discrepancies")
        and prior_validation.get("reference_version")
        in {"occupancy_dijkstra_v1", "independent_grid_dijkstra_v1"}
    )
    grids: dict[str, Any] = {}
    invalid_cases: set[str] = set()
    pending_by_map: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for case in manifest["cases"]:
        try:
            if case["map_path"] not in grids:
                grids[case["map_path"]] = _case_grid(manifest, case)
            cached = oracle_by_id.get(case["workload_id"])
            has_saved_reference = (
                reuse_manifest_references
                and "reference_cost" in case
                and "reference_reachable" in case
                and "reference_expanded" in case
            )
            if cached is None and has_saved_reference:
                cached = {
                    "schema_version": SCHEMA,
                    "workload_id": case["workload_id"],
                    "input": _input(case),
                    "outcome": {
                        "reference_cost": case["reference_cost"],
                        "reference_reachable": case["reference_reachable"],
                        "reference_expanded": case["reference_expanded"],
                        "reference_version": "occupancy_dijkstra_v1",
                    },
                }
                append_jsonl(oracle_path, cached, durable=False)
                oracle_by_id[case["workload_id"]] = cached
            if cached is None:
                pending_by_map[case["map_path"]].append(case)
        except Exception as exc:
            invalid_cases.add(case["workload_id"])
            discrepancies.append({"workload_id": case["workload_id"], "reason": str(exc)})

    batch_size = 16
    jobs = [
        (str(root), map_path, rows[offset : offset + batch_size])
        for map_path, rows in sorted(pending_by_map.items())
        for offset in range(0, len(rows), batch_size)
    ]
    workers = min(8, os.cpu_count() or 1, len(jobs))
    if jobs:
        with ProcessPoolExecutor(max_workers=workers, mp_context=mp.get_context("spawn")) as pool:
            for records in pool.map(_solve_oracle_batch, jobs):
                for record in records:
                    append_jsonl(oracle_path, record, durable=False)
                    oracle_by_id[record["workload_id"]] = record

    for case in manifest["cases"]:
        workload_id = case["workload_id"]
        if workload_id in invalid_cases:
            continue
        cached = oracle_by_id.get(workload_id)
        if cached is None:
            discrepancies.append({"workload_id": workload_id, "reason": "reference_solver_missing"})
            continue
        outcome = cached["outcome"]
        cost = outcome["reference_cost"]
        case["reference_cost"] = cost
        case["reference_reachable"] = outcome["reference_reachable"]
        case["reference_expanded"] = outcome["reference_expanded"]
        if cost is None or not scenario_matches_reference(case["scenario_optimum"], cost):
            discrepancies.append(
                {
                    "workload_id": workload_id,
                    "reason": "scenario_reference_mismatch",
                    "reference_cost": cost,
                }
            )
    manifest["validation"] = {
        "checked": len(manifest["cases"]),
        "discrepancies": discrepancies,
        "reference_version": "occupancy_dijkstra_v1",
        "oracle_cache": oracle_path.name,
        "oracle_cache_sha256": _file_sha(oracle_path),
        "oracle_workers": workers,
        "oracle_batch_size": batch_size,
    }
    manifest.pop("manifest_hash")
    manifest["manifest_hash"] = _sha(manifest)
    _write_json(manifest_path, manifest)
    return manifest["validation"]


class _PreparedGrid:
    """Share one native handle per variant while preparing a fresh h array per query."""

    def __init__(self, grid: Any, native_graph: Any) -> None:
        self.grid = grid
        self.native_graph = native_graph
        self.x_range = grid.width
        self.y_range = grid.height

    def to_native_graph(self) -> Any:
        return self.native_graph

    def native_heuristic_values(self, goal: int) -> np.ndarray:
        return self.grid.native_heuristic_values(goal)


def _peak_rss_bytes() -> int:
    maximum = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    return int(maximum if platform.system() == "Darwin" else maximum * 1024)


def _variant_params(variant: dict[str, Any]) -> dict[str, Any]:
    return {"weight": variant["weight"]} if variant["planner"] == "weighted_astar" else {}


def _path_ids(result: SearchResult) -> list[int] | None:
    if not result.success or not result.path_ids:
        return None
    return [int(result.path_ids[index]) for index in range(int(result.path_length))]


def _path_hash(path: list[int] | None) -> str | None:
    return _sha(path) if path is not None else None


def _native_call(
    library: Any,
    handle: Any,
    grid: Any,
    case: dict[str, Any],
    variant: dict[str, Any],
    *,
    measured: bool = False,
) -> dict[str, Any]:
    goal = int(case["goal"])
    h_start = time.perf_counter()
    heuristic = grid.native_heuristic_values(goal) if variant["weight"] else None
    h_time = time.perf_counter() - h_start
    options = SearchOptions(
        int(variant["algorithm"]),
        0,
        0,
        float(variant["weight"]),
        0,
        1,
        goal,
        ctypes.POINTER(ctypes.c_double)(),
        0,
    )
    heuristic_ptr = (
        heuristic.ctypes.data_as(ctypes.POINTER(ctypes.c_double))
        if heuristic is not None
        else ctypes.POINTER(ctypes.c_double)()
    )
    result = SearchResult()
    metrics = None
    try:
        started = time.perf_counter()
        if measured:
            from pathplanning.native._ffi import SearchMetrics

            metrics = SearchMetrics()
            metrics.struct_size = ctypes.sizeof(SearchMetrics)
            status = library.pp_native_search_plan_measured(
                handle,
                None,
                heuristic_ptr,
                int(case["start"]),
                ctypes.byref(options),
                ctypes.byref(result),
                ctypes.byref(metrics),
            )
        else:
            status = library.pp_native_search_plan(
                handle,
                None,
                heuristic_ptr,
                int(case["start"]),
                ctypes.byref(options),
                ctypes.byref(result),
            )
        native_s = time.perf_counter() - started
        if status or result.stop_reason == 3:
            message = (
                result.error_message.decode(errors="replace")
                if result.error_message
                else "native search failed"
            )
            raise RuntimeError(message)
        path = _path_ids(result)
        work = {}
        native_memory = {}
        if metrics is not None:
            fields = {
                name: int(getattr(metrics, name))
                for name, _ in metrics._fields_
                if name not in {"struct_size", "capability_bits"}
            }
            memory_fields = {
                "state_slots_allocated",
                "parent_id_bytes",
                "state_capacity_bytes_peak",
                "frontier_capacity_bytes_peak",
                "path_workspace_bytes_peak",
                "result_path_bytes",
                "query_workspace_peak_bytes",
                "native_requested_bytes_peak",
            }
            native_memory = {key: value for key, value in fields.items() if key in memory_fields}
            work = {key: value for key, value in fields.items() if key not in memory_fields}
            work["heuristic_array_values_prepared"] = (
                int(heuristic.size) if heuristic is not None else 0
            )
            work["reopen_count"] = None
            work["reexpanded_same_pass"] = None
            work["unsupported_reason"] = {
                "reopen_count": "kernel_does_not_reopen_closed_nodes",
                "reexpanded_same_pass": "kernel_does_not_reopen_closed_nodes",
            }
            capabilities = {
                "work_counters": bool(metrics.capability_bits & 1),
                "tracked_query_allocations": bool(metrics.capability_bits & 2),
                "supports_reopen": not bool(metrics.capability_bits & 4),
                "allocation_scope": "query_state_frontier_path_and_result_buffers",
                "allocation_limitations": "allocator_metadata_and_retained_graph_storage_excluded",
            }
        else:
            capabilities = {}
        return {
            "success": bool(result.success),
            "stop_reason": int(result.stop_reason),
            "iters": int(result.iters),
            "nodes": int(result.nodes),
            "path_cost": float(result.path_cost) if result.success else None,
            "path": path,
            "work": work,
            "native_memory": native_memory,
            "native_capabilities": capabilities,
            "timing": {"query_prepare_s": h_time, "native_call_s": native_s},
            "query_input_bytes": (int(heuristic.nbytes) if heuristic is not None else 0)
            + ctypes.sizeof(SearchOptions),
        }
    finally:
        library.pp_search_free_result(ctypes.byref(result))


def _public_call(
    grid: Any, native_graph: Any, case: dict[str, Any], variant: dict[str, Any]
) -> dict[str, Any]:
    source = _PreparedGrid(grid, native_graph)
    problem = DiscreteProblem(
        graph=source,
        start=int(case["start"]),
        goal=int(case["goal"]),
        params={"max_materialized_nodes": grid.node_count},
    )
    started = time.perf_counter()
    result = plan_discrete(
        problem, planner=variant["planner"], params=_variant_params(variant), seed=7
    )
    api_s = time.perf_counter() - started
    path = None if result.path is None else [int(value) for value in np.asarray(result.path).flat]
    return {
        "success": bool(result.success),
        "stop_reason": result.stop_reason.value,
        "iters": int(result.iters),
        "nodes": int(result.nodes),
        "path_cost": float(result.stats["path_cost"]) if result.success else None,
        "path": path,
        "work": {},
        "native_memory": {},
        "native_capabilities": {},
        "timing": {
            "api_total_s": api_s,
            "native_call_s": float(result.stats["native_search_s"]),
            "query_prepare_s": float(result.stats["graph_init_s"]),
            "result_decode_free_s": max(
                0.0,
                api_s
                - float(result.stats["graph_init_s"])
                - float(result.stats["native_search_s"]),
            ),
        },
        "query_input_bytes": (grid.node_count * 8 if variant["weight"] else 0)
        + ctypes.sizeof(SearchOptions),
    }


def _outcome(grid: Any, case: dict[str, Any], observed: dict[str, Any]) -> dict[str, Any]:
    oracle = case.get("reference_cost")
    path = observed["path"]
    validation = (
        validate_path(
            grid,
            path,
            int(case["start"]),
            int(case["goal"]),
            declared_cost=observed["path_cost"],
            oracle_cost=oracle,
        )
        if path is not None
        else None
    )
    valid = bool(validation.valid) if validation is not None else False
    if oracle is None:
        execution = "incorrect_unreachable_solution" if path else "proved_unreachable"
    elif not path:
        execution = "no_path"
    elif not valid or validation.declared_match is False:
        execution = "invalid_path"
    elif bool(validation.optimal_match):
        execution = "valid_optimal"
    else:
        execution = "valid_suboptimal"
    return {
        "execution_status": execution,
        "planner_stop_reason": observed["stop_reason"],
        "path_present": path is not None,
        "path_valid": valid if path is not None else None,
        "declared_cost_matches": validation.declared_match if validation else None,
        "optimal_cost_matches": validation.optimal_match if validation else None,
        "path_cost": observed["path_cost"],
        "oracle_cost": oracle,
        "path_length": len(path) if path is not None else None,
        "path_hash": _path_hash(path),
        "iters": observed["iters"],
        "nodes": observed["nodes"],
    }


def _input(case: dict[str, Any]) -> dict[str, Any]:
    return {
        "family": case["family"],
        "map_sha256": case["map_sha256"],
        "map_path": case["map_path"],
        "scenario_line": case["scenario_line"],
        "reference_cost": case.get("reference_cost"),
        "start": case["start"],
        "goal": case["goal"],
        "node_slots": case["node_slots"],
        "free_nodes": case["free_nodes"],
        "directed_edges": case["directed_edges"],
        "movement_profile": case.get("movement_profile", "land_octile_v1"),
        "difficulty_bin": case.get("difficulty_bin"),
    }


def _memory(grid: Any, observed: dict[str, Any], *, rss: bool) -> dict[str, Any]:
    return {
        **observed["native_memory"],
        "input_occupancy_bytes": int(grid.occupancy.nbytes),
        "query_input_bytes": observed["query_input_bytes"],
        "process_peak_rss_bytes": _peak_rss_bytes() if rss else None,
        "process_peak_rss_reason": None if rss else "different_measurement_pass",
    }


def _storage_info(library: Any, handle: Any) -> dict[str, int]:
    info = GraphStorageInfo()
    if library.pp_graph_get_storage_info(handle, ctypes.byref(info)):
        raise RuntimeError("native graph storage query failed")
    return {name: int(getattr(info, name)) for name, _ in info._fields_ if name != "struct_size"}


def _prepare_variant_graph(
    grid: Any, variant: dict[str, Any], graph_state: str
) -> tuple[Any, float, float, dict[str, int]]:
    started = time.perf_counter()
    graph = grid.to_native_graph()
    build_s = time.perf_counter() - started
    prep_started = time.perf_counter()
    if graph_state == "reused_graph" and variant["algorithm"] in (7, 9):
        library = load_native_library()
        message = ctypes.create_string_buffer(512)
        status = library.pp_graph_prepare_reverse(graph._native_handle, message, len(message))
        if status:
            raise RuntimeError(
                message.value.decode(errors="replace") or "reverse preparation failed"
            )
    prepare_s = time.perf_counter() - prep_started
    return graph, build_s, prepare_s, _storage_info(load_native_library(), graph._native_handle)


def _metrics_graph(release_graph: Any, variant: dict[str, Any]) -> tuple[Any, Any, float, float]:
    from pathplanning.native._ffi import load_search_metrics_library

    started = time.perf_counter()
    production = load_native_library()
    metrics_lib = load_search_metrics_library()
    view = GraphCsrView()
    if production.pp_graph_export_csr_view(release_graph._native_handle, ctypes.byref(view)):
        raise RuntimeError("could not export production CSR")
    graph = ctypes.c_void_p()
    message = ctypes.create_string_buffer(512)
    status = metrics_lib.pp_graph_create_csr_view(
        ctypes.byref(view),
        ctypes.byref(graph),
        message,
        len(message),
    )
    if status or graph.value is None:
        raise RuntimeError(message.value.decode(errors="replace") or "metrics CSR import failed")
    clone_s = time.perf_counter() - started
    prepare_started = time.perf_counter()
    if variant["algorithm"] in (7, 9):
        status = metrics_lib.pp_graph_prepare_reverse(graph, message, len(message))
        if status:
            metrics_lib.pp_graph_free(graph)
            raise RuntimeError(
                message.value.decode(errors="replace") or "metrics reverse preparation failed"
            )
    return metrics_lib, graph, clone_s, time.perf_counter() - prepare_started


def _execute_one(
    manifest: dict[str, Any],
    case: dict[str, Any],
    variant: dict[str, Any],
    pass_name: str,
    scope: str,
    graph_state: str,
    grid: Any,
    release_graph: Any,
    build_s: float,
    prepare_s: float,
    storage_info: dict[str, int],
    input_load_s: float,
    worker_retained_graph_bytes: int,
    metrics_workspace: tuple[Any, Any, float, float] | None = None,
) -> dict[str, Any]:
    memory_pass = pass_name == "memory"
    if pass_name == "work":
        production = load_native_library()
        release = _native_call(production, release_graph._native_handle, grid, case, variant)
        metrics_lib, metrics_graph, clone_s, metrics_prepare_s = (
            metrics_workspace
            if metrics_workspace is not None
            else _metrics_graph(release_graph, variant)
        )
        owns_metrics_graph = metrics_workspace is None
        try:
            instrumented = _native_call(
                metrics_lib, metrics_graph, grid, case, variant, measured=True
            )
        finally:
            if owns_metrics_graph:
                metrics_lib.pp_graph_free(metrics_graph)
        signature = ("success", "stop_reason", "iters", "nodes", "path_cost", "path")
        if any(release[key] != instrumented[key] for key in signature):
            raise RuntimeError("production/metrics parity mismatch")
        observed = instrumented
        observed["timing"]["metrics_clone_s"] = clone_s
        observed["timing"]["metrics_algorithm_prepare_s"] = metrics_prepare_s
        observed["timing"]["metrics_native_call_s"] = observed["timing"].pop("native_call_s")
        observed["timing"]["release_native_call_s_diagnostic"] = release["timing"]["native_call_s"]
    elif scope == "public_api":
        observed = _public_call(grid, release_graph, case, variant)
    else:
        observed = _native_call(
            load_native_library(), release_graph._native_handle, grid, case, variant
        )
    observed["timing"]["graph_build_s"] = build_s
    observed["timing"]["algorithm_prepare_s"] = prepare_s
    observed["timing"]["input_load_s"] = input_load_s
    if (
        scope == "public_api"
        and graph_state == "fresh_graph"
        and "api_total_s" in observed["timing"]
    ):
        observed["timing"]["api_total_s"] += build_s + prepare_s
    outcome = _outcome(grid, case, observed)
    storage_after = _storage_info(load_native_library(), release_graph._native_handle)
    if graph_state == "fresh_graph":
        worker_retained_graph_bytes = (
            storage_after["graph_object_bytes"]
            + storage_after["base_csr_capacity_bytes"]
            + storage_after["reverse_csr_capacity_bytes"]
        )
    return {
        "input": _input(case),
        "outcome": outcome,
        "work": observed["work"],
        "memory": {
            **_memory(grid, observed, rss=memory_pass),
            "worker_retained_graph_bytes": worker_retained_graph_bytes,
            **{f"{key}_before": value for key, value in storage_info.items()},
            **{f"{key}_after": value for key, value in storage_after.items()},
        },
        "timing": observed["timing"],
        "capabilities": {
            "supports_reopen": False,
            **observed["native_capabilities"],
            "work_counters": pass_name == "work",
            "rss_scope": "fresh_process_lifetime" if memory_pass else None,
        },
    }


def _worker(
    connection: Any,
    manifest: dict[str, Any],
    map_path: str,
    pass_name: str,
    scope: str,
    graph_state: str,
    initial_variant_id: str | None,
) -> None:
    """One map worker for latency/work; one query worker for memory."""
    metrics_graphs: dict[str, tuple[Any, Any, float, float]] = {}
    try:
        first = next(case for case in manifest["cases"] if case["map_path"] == map_path)
        input_started = time.perf_counter()
        grid = _case_grid(manifest, first)
        input_load_s = time.perf_counter() - input_started
        graphs: dict[str, tuple[Any, float, float, dict[str, int]]] = {}
        if graph_state == "reused_graph":
            for variant in manifest["variants"]:
                if initial_variant_id is not None and variant["variant_id"] != initial_variant_id:
                    continue
                graphs[variant["id"]] = _prepare_variant_graph(grid, variant, graph_state)
                if pass_name == "work":
                    metrics_graphs[variant["id"]] = _metrics_graph(
                        graphs[variant["id"]][0], variant
                    )
        worker_retained_graph_bytes = sum(
            storage["graph_object_bytes"]
            + storage["base_csr_capacity_bytes"]
            + storage["reverse_csr_capacity_bytes"]
            for _, _, _, storage in graphs.values()
        )
        connection.send({"kind": "READY", "map_path": map_path})
        while True:
            command = connection.recv()
            if command is None:
                break
            case, variant = command["case"], command["variant"]
            graph = None
            try:
                graph, build_s, prepare_s, storage_info = (
                    graphs[variant["id"]]
                    if graph_state == "reused_graph"
                    else _prepare_variant_graph(grid, variant, graph_state)
                )
                metrics_workspace = metrics_graphs.get(variant["id"])
                retained_bytes = worker_retained_graph_bytes
                value = _execute_one(
                    manifest,
                    case,
                    variant,
                    pass_name,
                    scope,
                    graph_state,
                    grid,
                    graph,
                    build_s,
                    prepare_s,
                    storage_info,
                    input_load_s,
                    retained_bytes,
                    metrics_workspace,
                )
                connection.send({"kind": "DONE", "value": value})
            except Exception as exc:
                connection.send(
                    {
                        "kind": "ERROR",
                        "error": f"{type(exc).__name__}: {exc}",
                        "traceback": traceback.format_exc(limit=4),
                    }
                )
            finally:
                if graph_state == "fresh_graph" and graph is not None:
                    graph.close()
    except Exception as exc:
        connection.send({"kind": "SETUP_ERROR", "error": f"{type(exc).__name__}: {exc}"})
    finally:
        for metrics_library, metrics_handle, _, _ in metrics_graphs.values():
            metrics_library.pp_graph_free(metrics_handle)
        connection.close()


def _schedule(
    manifest: dict[str, Any],
    pass_name: str,
    *,
    warmups: int,
    repeats: int,
    seed: int,
    scope: str,
    graph_state: str,
) -> list[dict[str, Any]]:
    rng = random.Random(seed)
    cases = manifest["cases"]
    if pass_name in ("latency", "memory"):
        ids = set(manifest["cohorts"][pass_name]["workload_ids"])
        cases = [case for case in cases if case["workload_id"] in ids]
    maps: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for case in cases:
        maps[case["map_path"]].append(case)
    schedule: list[dict[str, Any]] = []
    map_rows = sorted(maps.items())
    rng.shuffle(map_rows)
    for map_path, rows in map_rows:
        rows = list(rows)
        rng.shuffle(rows)
        for case in rows:
            phases = [("warmup", index) for index in range(warmups)] + [
                ("measured", index) for index in range(repeats)
            ]
            for phase, repetition in phases:
                variants = list(manifest["variants"])
                rng.shuffle(variants)
                for variant in variants:
                    schedule.append(
                        {
                            "map_path": map_path,
                            "workload_id": case["workload_id"],
                            "variant_id": variant["variant_id"],
                            "variant_name": variant["variant_name"],
                            "phase": phase,
                            "repetition": repetition,
                            "pass": pass_name,
                            "scope": scope,
                            "graph_state": graph_state,
                        }
                    )
    return schedule


def _complete_lines(path: Path) -> tuple[list[dict[str, Any]], bool, int]:
    if not path.exists():
        return [], False, 0
    raw = path.read_bytes()
    complete_end = raw.rfind(b"\n") + 1
    lines = raw[:complete_end].split(b"\n")[:-1]
    truncated = complete_end != len(raw)
    records = [json.loads(line) for line in lines if line]
    return records, truncated, complete_end


def _record_key(record: dict[str, Any]) -> tuple[Any, ...]:
    return (
        record["pass"],
        record["scope"],
        record["graph_state"],
        record["workload_id"],
        record["variant_id"],
        record["phase"],
        record["repetition"],
    )


_REQUIRED_WORK_METRICS = (
    "expanded",
    "discovered_first",
    "edges_examined",
    "relaxation_attempts",
    "relaxation_successes_first",
    "relaxation_successes_improved",
    "closed_neighbor_skips",
    "nonimproving_skips",
    "frontier_pushes",
    "frontier_pops",
    "stale_pops",
    "frontier_peak_entries",
    "goal_tests",
    "heuristic_lookups",
    "heuristic_computations",
    "validation_edge_checks",
)
_REQUIRED_MEMORY_METRICS = (
    "state_slots_allocated",
    "parent_id_bytes",
    "state_capacity_bytes_peak",
    "frontier_capacity_bytes_peak",
    "path_workspace_bytes_peak",
    "result_path_bytes",
    "query_workspace_peak_bytes",
    "native_requested_bytes_peak",
)


def run_campaign(
    manifest_path: str | Path,
    campaign_dir: str | Path,
    *,
    pass_name: str,
    scope: str = "public_api",
    graph_state: str = "reused_graph",
    repeats: int | None = None,
    warmups: int | None = None,
    schedule_seed: int = 7,
    query_timeout_s: float = 5.0,
    setup_timeout_s: float = 60.0,
) -> dict[str, Any]:
    """Run a pass with saved schedule, bounded workers, and resumable JSONL."""
    if pass_name not in {"latency", "work", "memory"}:
        raise ValueError("pass must be latency, work, or memory")
    if scope not in {"public_api", "prepared_kernel"}:
        raise ValueError("invalid scope")
    if graph_state not in {"fresh_graph", "reused_graph"}:
        raise ValueError("invalid graph state")
    manifest = load_manifest(manifest_path)
    validation = manifest.get("validation", {})
    if validation.get("discrepancies") or validation.get("checked") != len(manifest["cases"]):
        raise ValueError("manifest must pass validate before running")
    if repeats is None:
        repeats = 7 if pass_name == "latency" and manifest["profile"] == "pilot" else 1
    if warmups is None:
        warmups = 2 if pass_name == "latency" else 0
    if pass_name != "latency" and (repeats != 1 or warmups != 0):
        raise ValueError("work and memory passes use one observation per case/variant")
    campaign = Path(campaign_dir)
    campaign.mkdir(parents=True, exist_ok=True)
    _write_json(campaign / "manifest.json", manifest)
    oracle_source = Path(manifest_path).with_suffix(".oracle.jsonl")
    if oracle_source.is_file():
        shutil.copyfile(oracle_source, campaign / "oracle.jsonl")
    schedule = _schedule(
        manifest,
        pass_name,
        warmups=warmups,
        repeats=repeats,
        seed=schedule_seed,
        scope=scope,
        graph_state=graph_state,
    )
    protocol = {
        "pass": pass_name,
        "scope": scope,
        "graph_state": graph_state,
        "repeats": repeats,
        "warmups": warmups,
        "schedule_seed": schedule_seed,
        "query_timeout_s": query_timeout_s,
        "setup_timeout_s": setup_timeout_s,
    }
    root = Path(__file__).resolve().parents[2]
    native = root / "pathplanning" / "native"
    artifacts = sorted(
        [
            {"path": str(path.relative_to(root)), "sha256": _file_sha(path)}
            for path in native.glob("_search*engine*.so")
        ],
        key=lambda artifact: artifact["path"],
    )
    from scripts.benchmark_contract import environment_metadata, source_provenance

    provenance = {
        "source": source_provenance(campaign),
        "environment": environment_metadata(),
        "native_artifacts": artifacts,
    }
    identity = _sha(
        {
            "manifest": manifest["manifest_hash"],
            "protocol": protocol,
            "source": provenance["source"],
            "native": artifacts,
            "environment": provenance["environment"],
        }
    )
    host_identity = {
        key: provenance["environment"].get(key)
        for key in (
            "platform",
            "system",
            "machine",
            "processor",
            "logical_cpu_count",
            "python",
            "numpy",
        )
    }
    protocol_hash = stable_id("protocol", {"protocol": protocol, "host": host_identity})
    run_config = {
        "manifest_hash": manifest["manifest_hash"],
        "protocol": protocol,
        "protocol_hash": protocol_hash,
        "identity": identity,
        "provenance": provenance,
    }
    config_path = campaign / f"{pass_name}_{scope}_{graph_state}.json"
    if config_path.exists() and json.loads(config_path.read_text()) != run_config:
        raise ValueError("resume identity differs from saved campaign configuration")
    _write_json(config_path, run_config)
    schedule_path = campaign / f"schedule_{pass_name}_{scope}_{graph_state}.json"
    _write_json(schedule_path, schedule)
    runs_path = campaign / "runs.jsonl"
    truncated = recover_jsonl_tail(runs_path)
    prior, _, _ = _complete_lines(runs_path)
    prior_keys = {_record_key(row) for row in prior}
    if len(prior_keys) != len(prior):
        raise ValueError("duplicate observation keys in runs.jsonl")
    case_by_id = {case["workload_id"]: case for case in manifest["cases"]}
    variant_by_id = {variant["variant_id"]: variant for variant in manifest["variants"]}
    pending = [
        (index, entry)
        for index, entry in enumerate(schedule)
        if _record_key(entry) not in prior_keys
    ]
    context = mp.get_context("spawn")
    current_map = None
    worker = None
    connection = None
    counts: dict[str, int] = defaultdict(int)
    worker_setup_s = 0.0
    with runs_path.open("ab", buffering=0) as output:
        for order, entry in pending:
            map_path = entry["map_path"]
            failure_stage = None
            if current_map != map_path or pass_name == "memory" or worker is None:
                if connection is not None:
                    try:
                        connection.send(None)
                    except (BrokenPipeError, EOFError, OSError):
                        pass
                    connection.close()
                if worker is not None:
                    worker.join(timeout=1)
                    if worker.is_alive():
                        worker.kill()
                        worker.join()
                parent, child = context.Pipe()
                worker = context.Process(
                    target=_worker,
                    args=(
                        child,
                        manifest,
                        map_path,
                        pass_name,
                        scope,
                        graph_state,
                        entry["variant_id"] if pass_name == "memory" else None,
                    ),
                )
                setup_started = time.perf_counter()
                worker.start()
                child.close()
                connection = parent
                current_map = map_path
                if not parent.poll(setup_timeout_s):
                    worker.kill()
                    worker.join()
                    setup_message = {"kind": "SETUP_TIMEOUT"}
                else:
                    try:
                        setup_message = parent.recv()
                    except EOFError:
                        setup_message = {"kind": "SETUP_CRASH"}
                if setup_message["kind"] != "READY":
                    failure = setup_message.get("error", setup_message["kind"])
                    failure_stage = "setup"
                    worker_setup_s = time.perf_counter() - setup_started
                    worker = None
                    parent.close()
                    connection = None
                else:
                    failure = None
                    worker_setup_s = time.perf_counter() - setup_started
            else:
                failure = None
            started = time.perf_counter()
            value = None
            if failure is None and connection is not None:
                try:
                    connection.send(
                        {
                            "case": case_by_id[entry["workload_id"]],
                            "variant": variant_by_id[entry["variant_id"]],
                        }
                    )
                    if connection.poll(query_timeout_s):
                        reply = connection.recv()
                        if reply["kind"] == "DONE":
                            value = reply["value"]
                        else:
                            failure = reply.get("error", reply["kind"])
                            failure_stage = "query"
                    else:
                        failure = "query_timeout"
                        failure_stage = "query"
                        worker.kill()
                        worker.join()
                        worker = None
                        connection.close()
                        connection = None
                except (BrokenPipeError, EOFError, OSError) as exc:
                    failure = f"worker_crash: {exc}"
                    failure_stage = "query"
                    worker = None
                    connection.close()
                    connection = None
            elapsed = time.perf_counter() - started
            correctness_error = value is not None and value["outcome"]["execution_status"] in {
                "invalid_path",
                "incorrect_unreachable_solution",
                "no_path",
            }
            missing_metrics = []
            if value is not None and pass_name == "work":
                missing_metrics = [
                    name
                    for name in (*_REQUIRED_WORK_METRICS, *_REQUIRED_MEMORY_METRICS)
                    if value["work"].get(name) is None and value["memory"].get(name) is None
                ]
                if not value["capabilities"].get("work_counters"):
                    missing_metrics.append("work_counters_capability")
                if not value["capabilities"].get("tracked_query_allocations"):
                    missing_metrics.append("tracked_query_allocations_capability")
            status = (
                ("correctness_error" if correctness_error else "ok")
                if value is not None
                else ("timeout" if failure == "query_timeout" else "error")
            )
            if missing_metrics:
                status = "incomplete_measurements"
                failure = "missing_required_metrics:" + ",".join(missing_metrics)
            if correctness_error:
                failure = value["outcome"]["execution_status"]
            case = case_by_id[entry["workload_id"]]
            variant = variant_by_id[entry["variant_id"]]
            record = {
                "schema_version": SCHEMA,
                **entry,
                "order": order,
                "campaign_id": identity,
                "run_id": stable_id("run", [identity, list(_record_key(entry))]),
                "pass_name": pass_name,
                "protocol_hash": protocol_hash,
                "experiment_id": identity,
                "status": status,
                "input": _input(case),
                "outcome": value["outcome"]
                if value
                else {
                    "execution_status": status,
                    "planner_stop_reason": None,
                    "path_present": None,
                    "path_valid": None,
                    "oracle_cost": case.get("reference_cost"),
                },
                "work": value["work"] if value else {},
                "memory": value["memory"] if value else {},
                "timing": value["timing"] if value else {"elapsed_lower_bound_s": elapsed},
                "capabilities": {
                    **(value["capabilities"] if value else {}),
                    "missing_required_metrics": missing_metrics,
                },
                "variant_parameters": {
                    "planner": variant["planner"],
                    "weight": variant["weight"],
                    "heuristic_mode": "octile_precomputed_array" if variant["weight"] else "zero",
                    "tie_policy": "f_ascending_h_ascending_insertion_ascending",
                },
                "provenance": {"run_config": config_path.name},
                "error": failure,
                "error_stage": failure_stage,
            }
            if value is not None:
                record["timing"]["worker_setup_s"] = worker_setup_s
            validate_observation(record)
            output.write(_canonical(record) + b"\n")
            output.flush()
            counts[status] += 1
    if connection is not None:
        try:
            connection.send(None)
        except (BrokenPipeError, EOFError, OSError):
            pass
        connection.close()
    if worker is not None:
        worker.join(timeout=1)
        if worker.is_alive():
            worker.kill()
            worker.join()
    all_records, _, _ = _complete_lines(runs_path)
    scope_records = [
        row
        for row in all_records
        if (row.get("pass"), row.get("scope"), row.get("graph_state"))
        == (pass_name, scope, graph_state)
    ]
    current_counts: dict[str, int] = defaultdict(int)
    for row in scope_records:
        current_counts[str(row.get("status", "unknown"))] += 1
    expected_keys = {_record_key(entry) for entry in schedule}
    completed_keys = {_record_key(row) for row in scope_records}
    missing_keys = sorted(expected_keys - completed_keys, key=str)
    failures = sum(count for name, count in current_counts.items() if name != "ok")
    summary = {
        "expected": len(schedule),
        "completed": len(scope_records),
        "new": sum(counts.values()),
        "status_counts_new": dict(counts),
        "status_counts_campaign": dict(current_counts),
        "missing_observations": len(missing_keys),
        "failed_observations": failures,
        "complete": not missing_keys and failures == 0,
        "identity": identity,
        "truncated_line_recovered": truncated,
    }
    _write_json(campaign / f"run_summary_{pass_name}_{scope}_{graph_state}.json", summary)
    return summary
