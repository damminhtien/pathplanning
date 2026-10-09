"""Run source-aware planner cohorts for graph, voxel, and BARN assets.

Each format keeps its own query semantics. DIMACS inputs are directed weighted
graphs with zero heuristic; voxel grids use strict 26-neighbour Euclidean moves;
BARN is explicitly a derived point-robot XY cohort. OMPL.app resources are
catalogued here but require an external OMPL.app collision-checking runtime.
"""

from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import asdict
import gzip
import hashlib
import json
import math
from pathlib import Path
import time
from typing import Any, Iterable
import xml.etree.ElementTree as ET

import numpy as np
from scipy import sparse
from scipy.sparse.csgraph import connected_components, dijkstra

_ROOT = Path(__file__).resolve().parents[1]
_DISCRETE_VARIANTS: tuple[dict[str, Any], ...] = (
    {"name": "bfs", "params": {}},
    {"name": "dfs", "params": {}},
    {"name": "greedy_best_first", "params": {}},
    {"name": "astar", "params": {}},
    {"name": "dijkstra", "params": {}},
    {"name": "weighted_astar", "params": {"weight": 1.25}},
    {"name": "weighted_astar", "params": {"weight": 1.5}},
    {"name": "weighted_astar", "params": {"weight": 2.0}},
    {"name": "bidirectional_dijkstra", "params": {}},
    {"name": "bidirectional_astar", "params": {}},
    {"name": "anytime_astar", "params": {"anytime_weights": (2.0, 1.5, 1.25, 1.0)}},
    {"name": "reexp_astar", "params": {}},
    {"name": "dstar_lite", "params": {}},
)
_VOXEL_LIMIT = 3_000_000
_DIMACS_NODE_LIMIT = 2_000_000
_DIMACS_ARC_LIMIT = 4_000_000
_BARN_COLLISION_STEP = 0.01
_VOXEL_MOTIONS = tuple(
    (dx, dy, dz)
    for dz in (-1, 0, 1)
    for dy in (-1, 0, 1)
    for dx in (-1, 0, 1)
    if (dx, dy, dz) != (0, 0, 0)
)


def _sha_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def _stable_id(value: Any) -> str:
    return hashlib.sha256(
        json.dumps(value, sort_keys=True, separators=(",", ":")).encode()
    ).hexdigest()


def _asset_rows(
    dataset: dict[str, Any], index: dict[str, Any]
) -> list[dict[str, Any]]:
    return sorted(
        (index["assets"][asset_id] for asset_id in dataset["asset_ids"]),
        key=lambda asset: (asset["role"], asset["relative_path"]),
    )


def _checked_asset(root: Path, asset: dict[str, Any]) -> Path:
    path = (root / asset["relative_path"]).resolve()
    if not path.is_relative_to(root.resolve()) or not path.is_file():
        raise ValueError(f"missing or unsafe source asset: {asset['relative_path']}")
    if _sha_file(path) != asset["sha256"]:
        raise ValueError(f"source asset hash mismatch: {asset['relative_path']}")
    return path


def _dimacs_header(path: Path) -> tuple[int, int]:
    opener = gzip.open if path.suffix == ".gz" else open
    with opener(path, "rt", encoding="ascii") as stream:
        for line in stream:
            fields = line.split()
            if fields and fields[0] == "p":
                if len(fields) != 4 or fields[1] != "sp":
                    raise ValueError("invalid DIMACS shortest-path problem header")
                nodes, arcs = int(fields[2]), int(fields[3])
                if nodes < 1 or arcs < 0:
                    raise ValueError("invalid DIMACS graph dimensions")
                return nodes, arcs
    raise ValueError("missing DIMACS problem header")


def load_dimacs_csr(
    path: str | Path,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Read directed DIMACS arcs into CSR without coalescing parallel edges."""
    graph_path = Path(path)
    nodes, arcs = _dimacs_header(graph_path)
    sources = np.empty(arcs, dtype=np.int64)
    targets = np.empty(arcs, dtype=np.uint64)
    costs = np.empty(arcs, dtype=np.float64)
    opener = gzip.open if graph_path.suffix == ".gz" else open
    cursor = 0
    with opener(graph_path, "rt", encoding="ascii") as stream:
        for line in stream:
            fields = line.split()
            if not fields or fields[0] == "c" or fields[0] == "p":
                continue
            if fields[0] != "a" or len(fields) != 4 or cursor >= arcs:
                raise ValueError("invalid DIMACS arc record")
            source, target, cost = map(int, fields[1:])
            if not (1 <= source <= nodes and 1 <= target <= nodes):
                raise ValueError("DIMACS endpoint out of range")
            if cost < 0:
                raise ValueError("negative DIMACS costs are outside Dijkstra cohort")
            sources[cursor] = source - 1
            targets[cursor] = target - 1
            costs[cursor] = cost
            cursor += 1
    if cursor != arcs:
        raise ValueError(f"DIMACS arc count mismatch: expected {arcs}, read {cursor}")
    degrees = np.bincount(sources, minlength=nodes)
    offsets = np.empty(nodes + 1, dtype=np.uint64)
    offsets[0] = 0
    np.cumsum(degrees, out=offsets[1:])
    order = np.argsort(sources, kind="stable")
    indices = targets[order].astype(np.uint64, copy=False)
    weights = costs[order]
    return offsets, indices, weights


def _coalesce_parallel_arcs(
    offsets: np.ndarray, indices: np.ndarray, weights: np.ndarray
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Keep the minimum-cost arc for each ordered node pair."""
    if len(indices) == 0:
        return offsets.copy(), indices.copy(), weights.copy()
    node_count = len(offsets) - 1
    source_ids = np.repeat(
        np.arange(node_count, dtype=np.uint32),
        np.diff(offsets).astype(np.uint32, copy=False),
    )
    order = np.lexsort((indices, source_ids))
    sorted_sources = source_ids[order]
    sorted_targets = indices[order]
    group_start = np.empty(len(order), dtype=np.bool_)
    group_start[0] = True
    group_start[1:] = (sorted_sources[1:] != sorted_sources[:-1]) | (
        sorted_targets[1:] != sorted_targets[:-1]
    )
    starts = np.flatnonzero(group_start)
    unique_sources = sorted_sources[starts]
    unique_targets = sorted_targets[starts]
    minimum_weights = np.minimum.reduceat(weights[order], starts)
    degrees = np.bincount(unique_sources, minlength=node_count)
    new_offsets = np.empty(node_count + 1, dtype=np.uint64)
    new_offsets[0] = 0
    np.cumsum(degrees, out=new_offsets[1:])
    return new_offsets, unique_targets.astype(np.uint64, copy=False), minimum_weights


def load_voxel_map(path: str | Path) -> tuple[np.ndarray, tuple[int, int, int]]:
    """Decode MovingAI/Monash ``voxel`` and ``rev_voxel`` occupancy files."""
    map_path = Path(path)
    with map_path.open(encoding="ascii") as stream:
        header = stream.readline().split()
        if len(header) != 4 or header[0] not in {"voxel", "rev_voxel"}:
            raise ValueError("invalid voxel map header")
        width, height, depth = map(int, header[1:])
        if min(width, height, depth) <= 0:
            raise ValueError("voxel dimensions must be positive")
        slots = width * height * depth
        if slots > _VOXEL_LIMIT:
            raise ValueError(f"voxel grid exceeds configured node limit: {slots}")
        blocked = np.full(
            (depth, height, width), header[0] == "rev_voxel", dtype=np.bool_
        )
        for line in stream:
            if not line.strip():
                continue
            x, y, z = map(int, line.split())
            if not (0 <= x < width and 0 <= y < height and 0 <= z < depth):
                raise ValueError("voxel coordinate out of range")
            blocked[z, y, x] = header[0] == "voxel"
    return blocked, (width, height, depth)


def _proper_side_offsets(motion: tuple[int, int, int]) -> tuple[tuple[int, int, int], ...]:
    axes = [axis for axis, delta in enumerate(motion) if delta]
    sides = []
    for mask in range(1, (1 << len(axes)) - 1):
        offset = [0, 0, 0]
        for bit, axis in enumerate(axes):
            if mask & (1 << bit):
                offset[axis] = motion[axis]
        sides.append(tuple(offset))
    return tuple(sides)


def strict_voxel_csr(
    blocked: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Build symmetric 26-neighbour CSR; every proper diagonal side voxel is free."""
    if blocked.ndim != 3 or blocked.dtype != np.bool_:
        raise ValueError("blocked voxels must be a 3D boolean array")
    depth, height, width = blocked.shape
    count = width * height * depth
    valid = ~blocked.reshape(-1)
    node_ids = np.arange(count, dtype=np.int64)
    x = node_ids % width
    y = (node_ids // width) % height
    z = node_ids // (width * height)
    offsets_by_motion = [
        (motion, _proper_side_offsets(motion)) for motion in _VOXEL_MOTIONS
    ]

    def source_mask(motion: tuple[int, int, int], sides: tuple[tuple[int, int, int], ...]):
        dx, dy, dz = motion
        tx, ty, tz = x + dx, y + dy, z + dz
        inside = (
            (tx >= 0)
            & (tx < width)
            & (ty >= 0)
            & (ty < height)
            & (tz >= 0)
            & (tz < depth)
        )
        target = np.clip(node_ids + dx + width * dy + width * height * dz, 0, count - 1)
        mask = valid & inside & valid[target]
        for sx, sy, sz in sides:
            side = np.clip(
                node_ids + sx + width * sy + width * height * sz, 0, count - 1
            )
            mask &= valid[side]
        return mask

    degrees = np.zeros(count, dtype=np.uint8)
    for motion, sides in offsets_by_motion:
        degrees += source_mask(motion, sides)
    indptr = np.empty(count + 1, dtype=np.uint64)
    indptr[0] = 0
    np.cumsum(degrees, out=indptr[1:])
    edge_count = int(indptr[-1])
    indices = np.empty(edge_count, dtype=np.uint64)
    weights = np.empty(edge_count, dtype=np.float64)
    cursors = indptr[:-1].copy()
    for (dx, dy, dz), sides in offsets_by_motion:
        sources = np.flatnonzero(source_mask((dx, dy, dz), sides))
        positions = cursors[sources]
        indices[positions] = sources + dx + width * dy + width * height * dz
        weights[positions] = math.sqrt(dx * dx + dy * dy + dz * dz)
        cursors[sources] += 1
    return indptr, indices, weights


def _path_cost_and_validity(
    path: list[int] | None,
    indptr: np.ndarray,
    indices: np.ndarray,
    weights: np.ndarray,
) -> tuple[bool | None, float | None]:
    if path is None:
        return None, None
    total = 0.0
    for source, target in zip(path, path[1:]):
        begin, end = int(indptr[source]), int(indptr[source + 1])
        row = indices[begin:end]
        matches = np.flatnonzero(row == target)
        if not len(matches):
            return False, None
        total += float(np.min(weights[begin:end][matches]))
    return True, total


def _discrete_variants() -> Iterable[dict[str, Any]]:
    for variant in _DISCRETE_VARIANTS:
        yield {
            **variant,
            "variant_id": _stable_id([variant["name"], variant["params"], "zero_heuristic"]),
        }


def _run_discrete_case(
    *,
    dataset: dict[str, Any],
    cohort: str,
    query_id: str,
    graph: Any,
    indptr: np.ndarray,
    indices: np.ndarray,
    weights: np.ndarray,
    start: int,
    goal: int,
    source_assets: list[dict[str, Any]],
    seed: int,
    completed: set[tuple[str, str]],
    output_stream: Any,
    input_metadata: dict[str, Any] | None = None,
) -> list[dict[str, Any]]:
    from pathplanning.api import plan_discrete
    from pathplanning.core.contracts import DiscreteProblem

    matrix = sparse.csr_matrix((weights, indices, indptr), shape=(graph.node_count, graph.node_count))
    oracle = float(dijkstra(matrix, directed=True, indices=start)[goal])
    oracle_cost = oracle if math.isfinite(oracle) else None
    metadata = dict(input_metadata or {})
    source_reference = metadata.get("source_scenario_optimum")
    source_reference_matches_oracle = (
        math.isclose(float(source_reference), oracle_cost, rel_tol=1e-8, abs_tol=1e-7)
        if source_reference is not None and oracle_cost is not None
        else None
    )
    problem = DiscreteProblem(graph=graph, start=start, goal=goal)
    results = []
    for variant in _discrete_variants():
        variant_id = variant["variant_id"]
        if (dataset["dataset_id"], variant_id) in completed:
            continue
        started = time.perf_counter()
        error = None
        try:
            result = plan_discrete(
                problem,
                planner=variant["name"],
                params=variant["params"],
                seed=seed,
            )
            elapsed = time.perf_counter() - started
            path = (
                [int(value) for value in np.asarray(result.path).reshape(-1)]
                if result.path is not None
                else None
            )
            path_valid, reconstructed_cost = _path_cost_and_validity(
                path, indptr, indices, weights
            )
            declared_cost = result.stats.get("path_cost")
            valid = bool(path_valid) and (
                declared_cost is None
                or reconstructed_cost is not None
                and math.isclose(float(declared_cost), reconstructed_cost, rel_tol=1e-10, abs_tol=1e-8)
            )
            if oracle_cost is None:
                outcome = "proved_unreachable" if path is None else "incorrect_unreachable_solution"
            elif path is None:
                outcome = "no_path"
            elif not valid:
                outcome = "invalid_path"
            elif math.isclose(reconstructed_cost or 0.0, oracle_cost, rel_tol=1e-10, abs_tol=1e-8):
                outcome = "valid_optimal"
            else:
                outcome = "valid_suboptimal"
            row = {
                "schema_version": "pathplanning_format_cohort_v1",
                "source": dataset["source"],
                "family": dataset["family"],
                "dataset_id": dataset["dataset_id"],
                "dataset_name": dataset["name"],
                "cohort": cohort,
                "query_id": query_id,
                "variant_id": variant_id,
                "planner": variant["name"],
                "parameters": variant["params"],
                "seed": seed,
                "input": {
                    "start": start,
                    "goal": goal,
                    "node_count": graph.node_count,
                    "edge_count": len(indices),
                    "source_asset_hashes": [asset["sha256"] for asset in source_assets],
                    "heuristic": "zero_no_source_spatial_lower_bound",
                    "source_scenario_reference_matches_oracle": source_reference_matches_oracle,
                    **metadata,
                },
                "outcome": {
                    "status": outcome,
                    "path_valid": valid if path is not None else None,
                    "path_cost": reconstructed_cost,
                    "oracle_cost": oracle_cost,
                    "optimal": outcome == "valid_optimal",
                    "iters": int(result.iters),
                    "nodes": int(result.nodes),
                },
                "timing": {"public_api_s": elapsed},
                "error": None,
            }
        except Exception as exc:
            elapsed = time.perf_counter() - started
            error = f"{type(exc).__name__}: {exc}"
            row = {
                "schema_version": "pathplanning_format_cohort_v1",
                "source": dataset["source"],
                "family": dataset["family"],
                "dataset_id": dataset["dataset_id"],
                "dataset_name": dataset["name"],
                "cohort": cohort,
                "query_id": query_id,
                "variant_id": variant_id,
                "planner": variant["name"],
                "parameters": variant["params"],
                "seed": seed,
                "input": {
                    "start": start,
                    "goal": goal,
                    "node_count": graph.node_count,
                    "edge_count": len(indices),
                    "source_asset_hashes": [asset["sha256"] for asset in source_assets],
                    "heuristic": "zero_no_source_spatial_lower_bound",
                    "source_scenario_reference_matches_oracle": source_reference_matches_oracle,
                    **metadata,
                },
                "outcome": {"status": "error", "path_valid": None, "oracle_cost": oracle_cost},
                "timing": {"public_api_s": elapsed},
                "error": error,
            }
        output_stream.write(json.dumps(row, sort_keys=True, separators=(",", ":")) + "\n")
        output_stream.flush()
        results.append(row)
        completed.add((dataset["dataset_id"], variant_id))
    return results


def _select_dimacs_query(
    indptr: np.ndarray, indices: np.ndarray, node_count: int, seed: int
) -> tuple[int, int] | None:
    adjacency = sparse.csr_matrix(
        (np.ones(len(indices), dtype=np.uint8), indices, indptr), shape=(node_count, node_count)
    )
    count, labels = connected_components(adjacency, directed=True, connection="strong")
    sizes = np.bincount(labels, minlength=count)
    eligible = np.flatnonzero(sizes >= 2)
    if not len(eligible):
        return None
    component = int(eligible[np.argmax(sizes[eligible])])
    nodes = np.flatnonzero(labels == component)
    rng = np.random.default_rng(seed)
    pair = rng.choice(nodes, size=2, replace=False)
    return int(pair[0]), int(pair[1])


def _voxel_scenario_query(
    path: Path, dimensions: tuple[int, int, int], blocked: np.ndarray, seed: int
) -> tuple[int, int, str, dict[str, Any]] | None:
    width, height, depth = dimensions
    candidates = []
    with path.open(encoding="ascii") as stream:
        version = stream.readline().strip()
        if version not in {"version 1", "version 2"}:
            raise ValueError("unsupported voxel scenario version")
        _map_name = stream.readline().strip()
        for line_number, line in enumerate(stream, start=3):
            fields = line.split()
            if not fields:
                continue
            if len(fields) != (8 if version == "version 1" else 9):
                raise ValueError("invalid voxel scenario row")
            coordinates = tuple(map(int, fields[:6]))
            sx, sy, sz, gx, gy, gz = coordinates
            if not (
                0 <= sx < width
                and 0 <= gx < width
                and 0 <= sy < height
                and 0 <= gy < height
                and 0 <= sz < depth
                and 0 <= gz < depth
            ):
                raise ValueError("voxel scenario endpoint out of bounds")
            if blocked[sz, sy, sx] or blocked[gz, gy, gx]:
                continue
            start = sx + width * (sy + height * sz)
            goal = gx + width * (gy + height * gz)
            normalized = math.dist((sx, sy, sz), (gx, gy, gz)) / max(
                1.0,
                math.sqrt((width - 1) ** 2 + (height - 1) ** 2 + (depth - 1) ** 2),
            )
            if start != goal:
                source_optimum = float(fields[6])
                difficulty = float(fields[7])
                if not math.isfinite(source_optimum) or source_optimum < 0:
                    raise ValueError("invalid voxel scenario optimum")
                if not math.isfinite(difficulty):
                    raise ValueError("invalid voxel scenario difficulty")
                candidates.append(
                    (normalized, start, goal, line_number, source_optimum, difficulty)
                )
    if not candidates:
        return None
    maximum = max(item[0] for item in candidates)
    far_candidates = [item for item in candidates if item[0] == maximum]
    selected = far_candidates[seed % len(far_candidates)]
    query_id = _stable_id(
        ["voxel_scenario", selected[1], selected[2], selected[3], seed]
    )
    return selected[1], selected[2], query_id, {
        "scenario_line": selected[3],
        "source_scenario_optimum": selected[4],
        "source_scenario_difficulty": selected[5],
        "movement_semantics": "strict_26_neighbor_euclidean_no_corner_cutting",
    }


def _barn_geometry(path: Path) -> tuple[list[tuple[float, float, float]], np.ndarray]:
    root = ET.parse(path).getroot()
    circles: list[tuple[float, float, float]] = []

    def pose(element: ET.Element) -> tuple[float, float, float]:
        values = [float(value) for value in (element.findtext("pose") or "0 0 0 0 0 0").split()]
        if len(values) != 6 or not all(math.isfinite(value) for value in values) or any(values[3:5]):
            raise ValueError("unsupported or invalid BARN collision pose")
        return values[0], values[1], values[5]

    def compose(parent: tuple[float, float, float], child: tuple[float, float, float]):
        x, y, angle = parent
        dx, dy, yaw = child
        return (
            x + math.cos(angle) * dx - math.sin(angle) * dy,
            y + math.sin(angle) * dx + math.cos(angle) * dy,
            angle + yaw,
        )

    for model in root.findall(".//world/model"):
        for link in model.findall("link"):
            for collision in link.findall("collision"):
                radius_text = collision.findtext("geometry/cylinder/radius")
                if radius_text is None:
                    if collision.find("geometry/plane") is not None:
                        continue
                    raise ValueError("unsupported BARN collision geometry")
                center = compose(compose(pose(model), pose(link)), pose(collision))
                radius = float(radius_text)
                if not math.isfinite(radius) or radius <= 0:
                    raise ValueError("invalid BARN cylinder radius")
                circles.append((center[0], center[1], radius))
    if not circles:
        raise ValueError("BARN world has no collision cylinders")
    values = np.asarray(circles, dtype=np.float64)
    bounds = np.vstack(
        (
            np.min(values[:, :2] - values[:, 2, None], axis=0),
            np.max(values[:, :2] + values[:, 2, None], axis=0),
        )
    )
    return circles, bounds


def _barn_planning_circles(
    circles: list[tuple[float, float, float]], collision_step: float
) -> tuple[list[tuple[float, float, float]], float]:
    """Inflate obstacles enough that sampled checks cannot miss source collisions."""
    step = float(collision_step)
    if not math.isfinite(step) or step <= 0.0:
        raise ValueError("collision_step must be > 0")
    margin = step / 2.0 + 1e-9
    return [(cx, cy, radius + margin) for cx, cy, radius in circles], margin


def _segment_clear(
    first: np.ndarray,
    second: np.ndarray,
    circles: list[tuple[float, float, float]],
) -> np.ndarray:
    delta = second - first
    denominator = np.sum(delta * delta, axis=1)
    clear = np.ones(len(first), dtype=np.bool_)
    for cx, cy, radius in circles:
        center = np.array([cx, cy])
        amount = np.divide(
            np.sum((center - first) * delta, axis=1),
            denominator,
            out=np.zeros(len(first), dtype=np.float64),
            where=denominator > 0,
        )
        nearest = first + np.clip(amount, 0.0, 1.0)[:, None] * delta
        clear &= np.sum((nearest - center) ** 2, axis=1) > radius**2
    return clear


def _barn_query(
    circles: list[tuple[float, float, float]],
    bounds: np.ndarray,
    seed: int,
    *,
    resolution: int = 64,
) -> tuple[np.ndarray, np.ndarray, dict[str, Any]] | None:
    step = (bounds[1] - bounds[0]) / resolution
    xs = bounds[0, 0] + (np.arange(resolution) + 0.5) * step[0]
    ys = bounds[0, 1] + (np.arange(resolution) + 0.5) * step[1]
    xx, yy = np.meshgrid(xs, ys)
    points = np.column_stack((xx.reshape(-1), yy.reshape(-1)))
    valid = np.ones(len(points), dtype=np.bool_)
    for cx, cy, radius in circles:
        valid &= np.sum((points - (cx, cy)) ** 2, axis=1) > radius**2
    grid_ids = np.arange(resolution * resolution).reshape(resolution, resolution)
    right_source = grid_ids[:, :-1].reshape(-1)
    right_target = grid_ids[:, 1:].reshape(-1)
    down_source = grid_ids[:-1, :].reshape(-1)
    down_target = grid_ids[1:, :].reshape(-1)
    sources = np.concatenate((right_source, down_source))
    targets = np.concatenate((right_target, down_target))
    eligible = valid[sources] & valid[targets]
    sources, targets = sources[eligible], targets[eligible]
    clear = _segment_clear(points[sources], points[targets], circles)
    sources, targets = sources[clear], targets[clear]
    if not len(sources):
        return None
    adjacency = sparse.coo_matrix(
        (
            np.ones(2 * len(sources), dtype=np.uint8),
            (np.concatenate((sources, targets)), np.concatenate((targets, sources))),
        ),
        shape=(len(points), len(points)),
    ).tocsr()
    count, labels = connected_components(adjacency, directed=False)
    sizes = np.bincount(labels, minlength=count)
    eligible_components = np.flatnonzero(sizes >= 2)
    if not len(eligible_components):
        return None
    component = int(eligible_components[np.argmax(sizes[eligible_components])])
    nodes = np.flatnonzero(labels == component)
    rng = np.random.default_rng(seed)
    sampled = rng.choice(nodes, size=min(384, len(nodes)), replace=False)
    states = points[sampled]
    distances = np.sum((states[:, None, :] - states[None, :, :]) ** 2, axis=2)
    first_index, second_index = np.unravel_index(int(np.argmax(distances)), distances.shape)
    start, goal = states[first_index].copy(), states[second_index].copy()
    return start, goal, {
        "grid_resolution": resolution,
        "free_grid_cells": int(valid.sum()),
        "connected_components": int(count),
        "selected_component_cells": int(sizes[component]),
        "query_distance": float(np.linalg.norm(goal - start)),
        "reachability_certificate": "collision_checked_point_robot_grid_path",
    }


def _continuous_variants() -> list[str]:
    from pathplanning.registry import list_planners

    unsupported = {"hybrid_astar", "jit_star", "state_lattice"}
    return [name for name in list_planners("continuous") if name not in unsupported]


def _continuous_params(planner: str) -> Any:
    from pathplanning.core.params import RitParams, RoadmapParams, RrtParams

    common = {
        "max_iters": 3_000,
        "step_size": 0.5,
        "goal_sample_rate": 0.05,
        "time_budget_s": None,
        "max_sample_tries": 1_000,
        "collision_step": _BARN_COLLISION_STEP,
        "sample_count": 512,
        "batch_size": 64,
        "allow_python_callbacks": False,
    }
    if planner in {"prm_star", "lazy_prm", "eirm_star"}:
        return RoadmapParams(
            sample_count=512,
            gamma=8.0,
            max_sample_tries=1_000,
            max_expansions=100_000,
            collision_step=_BARN_COLLISION_STEP,
            allow_python_callbacks=False,
        )
    if planner == "rit_star":
        return RitParams(**{key: value for key, value in common.items() if key not in {"goal_sample_rate"}}, gamma=8.0)
    return RrtParams(**common)


def _run_barn_dataset(
    *,
    root: Path,
    index: dict[str, Any],
    output_stream: Any,
    completed: set[tuple[str, str]],
) -> list[dict[str, Any]]:
    from pathplanning.api import plan_continuous
    from pathplanning.core.contracts import ContinuousProblem, GoalState
    from pathplanning.spaces.grid2d import Grid2DSamplingSpace

    results = []
    datasets = [row for row in index["datasets"].values() if row["source"] == "barn"]
    planners = _continuous_variants()
    for dataset in sorted(datasets, key=lambda row: row["dataset_id"]):
        assets = _asset_rows(dataset, index)
        world_asset = next((asset for asset in assets if asset["role"] == "dataset"), None)
        path_asset = next((asset for asset in assets if asset["role"] == "supplied_path"), None)
        if world_asset is None:
            continue
        try:
            world_path = _checked_asset(root, world_asset)
            circles, bounds = _barn_geometry(world_path)
            planning_circles, collision_margin = _barn_planning_circles(
                circles, _BARN_COLLISION_STEP
            )
            seed = int(dataset["dataset_id"][:8], 16)
            query = _barn_query(planning_circles, bounds, seed)
            if query is None:
                raise ValueError("no connected point-XY query found on collision-checked grid")
            start, goal, query_metadata = query
            space = Grid2DSamplingSpace(
                x_range=(float(bounds[0, 0]), float(bounds[1, 0])),
                y_range=(float(bounds[0, 1]), float(bounds[1, 1])),
                obs_circle=[list(circle) for circle in planning_circles],
                delta=0.0,
                collision_step=_BARN_COLLISION_STEP,
                max_sample_tries=10_000,
            )
            problem = ContinuousProblem(
                space=space,
                start=start,
                goal=GoalState(state=goal),
            )
            asset_hashes = [world_asset["sha256"]]
            if path_asset is not None:
                _checked_asset(root, path_asset)
                asset_hashes.append(path_asset["sha256"])
            for planner in planners:
                params = _continuous_params(planner)
                params_dict = asdict(params)
                variant_id = _stable_id([planner, "point_xy", params_dict])
                key = (dataset["dataset_id"], variant_id)
                if key in completed:
                    continue
                started = time.perf_counter()
                try:
                    plan = plan_continuous(
                        problem,
                        planner=planner,
                        params=params,
                        seed=seed,
                    )
                    elapsed = time.perf_counter() - started
                    path = (
                        np.asarray(plan.path, dtype=np.float64).reshape(-1, 2)
                        if plan.path is not None
                        else None
                    )
                    path_valid = None
                    length = None
                    if path is not None and len(path) >= 2:
                        path_valid = (
                            np.allclose(path[0], start, atol=1e-7, rtol=0.0)
                            and np.allclose(path[-1], goal, atol=1e-7, rtol=0.0)
                            and bool(np.all(_segment_clear(path[:-1], path[1:], circles)))
                        )
                        length = float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum())
                    outcome = (
                        "no_solution_found"
                        if path is None
                        else "invalid_path"
                        if not path_valid
                        else "valid_path"
                        if bool(plan.success)
                        else "valid_path_reported_failure"
                    )
                    row = {
                        "schema_version": "pathplanning_format_cohort_v1",
                        "source": "barn",
                        "family": dataset["family"],
                        "dataset_id": dataset["dataset_id"],
                        "dataset_name": dataset["name"],
                        "cohort": "barn_point_xy_derived",
                        "query_id": _stable_id([dataset["dataset_id"], start.tolist(), goal.tolist()]),
                        "variant_id": variant_id,
                        "planner": planner,
                        "seed": seed,
                        "input": {
                            "start": start.tolist(),
                            "goal": goal.tolist(),
                            "bounds": bounds.tolist(),
                            "obstacle_count": len(circles),
                            "robot_model": "point_xy_derived",
                            "goal_semantics": "exact_goal_state",
                            "collision_step": _BARN_COLLISION_STEP,
                            "planning_obstacle_margin_m": collision_margin,
                            "collision_checker": "native_sampling_with_half_step_obstacle_margin",
                            "path_validation_geometry": "exact_source_circles",
                            "source_path_asset_sha256": path_asset["sha256"] if path_asset else None,
                            "source_path_coordinates": "not_used_row_column_not_world_xy",
                            "source_asset_hashes": asset_hashes,
                            **query_metadata,
                        },
                        "outcome": {
                            "status": outcome,
                            "path_valid": path_valid,
                            "path_length": length,
                            "optimality": "unavailable_no_independent_continuous_oracle",
                            "nodes": int(plan.nodes),
                            "iters": int(plan.iters),
                        },
                        "timing": {"public_api_s": elapsed},
                        "parameters": params_dict,
                        "error": None,
                    }
                except Exception as exc:
                    elapsed = time.perf_counter() - started
                    row = {
                        "schema_version": "pathplanning_format_cohort_v1",
                        "source": "barn",
                        "family": dataset["family"],
                        "dataset_id": dataset["dataset_id"],
                        "dataset_name": dataset["name"],
                        "cohort": "barn_point_xy_derived",
                        "query_id": _stable_id([dataset["dataset_id"], start.tolist(), goal.tolist()]),
                        "variant_id": variant_id,
                        "planner": planner,
                        "input": {"source_asset_hashes": asset_hashes},
                        "outcome": {"status": "error", "path_valid": None},
                        "timing": {"public_api_s": elapsed},
                        "error": f"{type(exc).__name__}: {exc}",
                    }
                output_stream.write(json.dumps(row, sort_keys=True, separators=(",", ":")) + "\n")
                output_stream.flush()
                results.append(row)
                completed.add(key)
        except Exception as exc:
            results.append(
                {
                    "source": "barn",
                    "dataset_id": dataset["dataset_id"],
                    "dataset_name": dataset["name"],
                    "status": "input_error",
                    "error": f"{type(exc).__name__}: {exc}",
                }
            )
    return results


def _run_dimacs_dataset(
    *,
    root: Path,
    index: dict[str, Any],
    output_stream: Any,
    completed: set[tuple[str, str]],
) -> list[dict[str, Any]]:
    from pathplanning.native import NativeGraph

    results = []
    datasets = [row for row in index["datasets"].values() if row["source"] == "dimacs"]
    for dataset in sorted(datasets, key=lambda row: row["dataset_id"]):
        assets = _asset_rows(dataset, index)
        graph_asset = next((asset for asset in assets if asset["role"] == "dataset"), None)
        if graph_asset is None:
            continue
        try:
            path = _checked_asset(root, graph_asset)
            node_count, arc_count = _dimacs_header(path)
            if node_count > _DIMACS_NODE_LIMIT or arc_count > _DIMACS_ARC_LIMIT:
                results.append(
                    {
                        "source": "dimacs",
                        "dataset_id": dataset["dataset_id"],
                        "dataset_name": dataset["name"],
                        "family": dataset["family"],
                        "status": "resource_limited",
                        "node_count": node_count,
                        "arc_count": arc_count,
                        "reason": "configured_graph_size_limit",
                    }
                )
                continue
            offsets, indices, weights = _coalesce_parallel_arcs(*load_dimacs_csr(path))
            graph = NativeGraph.from_csr(offsets, indices, weights)
            try:
                seed = int(dataset["dataset_id"][:8], 16)
                selected = _select_dimacs_query(offsets, indices, node_count, seed)
                if selected is None:
                    results.append({"source": "dimacs", "dataset_id": dataset["dataset_id"], "status": "no_strongly_connected_pair"})
                    continue
                query_id = _stable_id([dataset["dataset_id"], selected, dataset["family"]])
                _run_discrete_case(
                    dataset=dataset,
                    cohort=f"dimacs_{dataset['family']}_directed_weighted_graph",
                    query_id=query_id,
                    graph=graph,
                    indptr=offsets,
                    indices=indices,
                    weights=weights,
                    start=selected[0],
                    goal=selected[1],
                    source_assets=[graph_asset],
                    seed=seed,
                    completed=completed,
                    output_stream=output_stream,
                    input_metadata={
                        "parallel_arc_policy": "minimum_cost_per_ordered_pair",
                        "source_arc_count": arc_count,
                        "search_arc_count": len(indices),
                    },
                )
                results.append(
                    {
                        "source": "dimacs",
                        "dataset_id": dataset["dataset_id"],
                        "dataset_name": dataset["name"],
                        "family": dataset["family"],
                        "status": "completed",
                        "node_count": node_count,
                        "arc_count": arc_count,
                        "query_id": query_id,
                    }
                )
            finally:
                graph.close()
        except Exception as exc:
            results.append(
                {
                    "source": "dimacs",
                    "dataset_id": dataset["dataset_id"],
                    "dataset_name": dataset["name"],
                    "family": dataset["family"],
                    "status": "input_error",
                    "error": f"{type(exc).__name__}: {exc}",
                }
            )
    return results


def _run_voxel_datasets(
    *,
    root: Path,
    index: dict[str, Any],
    output_stream: Any,
    completed: set[tuple[str, str]],
) -> list[dict[str, Any]]:
    from pathplanning.native import NativeGraph

    results = []
    datasets = [
        row
        for row in index["datasets"].values()
        if row["representation"] == "voxel3d" and row["source"] in {"movingai", "monash"}
    ]
    seen_map_hashes: dict[str, str] = {}
    for dataset in sorted(datasets, key=lambda row: (row["source"] != "movingai", row["dataset_id"])):
        assets = _asset_rows(dataset, index)
        map_asset = next((asset for asset in assets if asset["role"] == "dataset"), None)
        scenario_asset = next((asset for asset in assets if asset["role"] == "scenario"), None)
        if map_asset is None:
            continue
        prior_source = seen_map_hashes.get(map_asset["sha256"])
        if prior_source is not None:
            results.append(
                {
                    "source": dataset["source"],
                    "dataset_id": dataset["dataset_id"],
                    "dataset_name": dataset["name"],
                    "family": dataset["family"],
                    "status": "mirror_duplicate",
                    "canonical_source": prior_source,
                    "sha256": map_asset["sha256"],
                }
            )
            continue
        seen_map_hashes[map_asset["sha256"]] = dataset["source"]
        try:
            map_path = _checked_asset(root, map_asset)
            with map_path.open(encoding="ascii") as stream:
                header = stream.readline().split()
            if len(header) != 4 or header[0] not in {"voxel", "rev_voxel"}:
                raise ValueError("invalid voxel map header")
            dimensions = tuple(map(int, header[1:]))
            slots = math.prod(dimensions)
            if slots > _VOXEL_LIMIT:
                results.append(
                    {
                        "source": dataset["source"],
                        "dataset_id": dataset["dataset_id"],
                        "dataset_name": dataset["name"],
                        "family": dataset["family"],
                        "status": "resource_limited",
                        "node_count": slots,
                        "reason": "configured_voxel_node_limit",
                    }
                )
                continue
            if scenario_asset is None:
                results.append(
                    {
                        "source": dataset["source"],
                        "dataset_id": dataset["dataset_id"],
                        "dataset_name": dataset["name"],
                        "status": "no_source_scenario",
                    }
                )
                continue
            scenario_path = _checked_asset(root, scenario_asset)
            blocked, dimensions = load_voxel_map(map_path)
            query = _voxel_scenario_query(
                scenario_path,
                dimensions,
                blocked,
                int(dataset["dataset_id"][:8], 16),
            )
            if query is None:
                results.append(
                    {
                        "source": dataset["source"],
                        "dataset_id": dataset["dataset_id"],
                        "dataset_name": dataset["name"],
                        "status": "no_valid_source_query",
                    }
                )
                continue
            start, goal, query_id, source_metadata = query
            offsets, indices, weights = strict_voxel_csr(blocked)
            graph = NativeGraph.from_csr(offsets, indices, weights)
            try:
                _run_discrete_case(
                    dataset=dataset,
                    cohort=f"{dataset['source']}_{dataset['family']}_strict_26_euclidean",
                    query_id=query_id,
                    graph=graph,
                    indptr=offsets,
                    indices=indices,
                    weights=weights,
                    start=start,
                    goal=goal,
                    source_assets=[map_asset, scenario_asset],
                    seed=int(dataset["dataset_id"][:8], 16),
                    completed=completed,
                    input_metadata=source_metadata,
                    output_stream=output_stream,
                )
            finally:
                graph.close()
            results.append(
                {
                    "source": dataset["source"],
                    "dataset_id": dataset["dataset_id"],
                    "dataset_name": dataset["name"],
                    "family": dataset["family"],
                    "status": "completed",
                    "node_count": slots,
                    "edge_count": len(indices),
                    "query_id": query_id,
                }
            )
        except Exception as exc:
            results.append(
                {
                    "source": dataset["source"],
                    "dataset_id": dataset["dataset_id"],
                    "dataset_name": dataset["name"],
                    "family": dataset["family"],
                    "status": "input_error",
                    "error": f"{type(exc).__name__}: {exc}",
                }
            )
    return results


def _read_jsonl(path: Path) -> list[dict[str, Any]]:
    if not path.is_file():
        return []
    content = path.read_bytes()
    complete_end = content.rfind(b"\n") + 1
    if complete_end != len(content):
        with path.open("r+b") as stream:
            stream.truncate(complete_end)
    return [
        json.loads(line)
        for line in content[:complete_end].decode("utf-8").splitlines()
        if line.strip()
    ]


def run_format_cohorts(
    dataset_root: str | Path,
    dataset_index: str | Path,
    output: str | Path,
    *,
    sources: Iterable[str] = ("dimacs", "voxels", "barn"),
) -> dict[str, Any]:
    """Resume per-format public-API cohorts and save coverage/provenance."""
    root = Path(dataset_root).resolve()
    index_path = Path(dataset_index).resolve()
    index = json.loads(index_path.read_text())
    if index.get("schema_version") != "pathplanning_dataset_installation_v1":
        raise ValueError("unsupported dataset index schema")
    campaign = Path(output).resolve()
    campaign.mkdir(parents=True, exist_ok=True)
    runs_path = campaign / "runs.jsonl"
    prior = _read_jsonl(runs_path)
    completed = {
        (row["dataset_id"], row["variant_id"])
        for row in prior
        if row.get("schema_version") == "pathplanning_format_cohort_v1"
        and row.get("outcome", {}).get("status") != "error"
        and row.get("variant_id")
    }
    selectors = set(sources)
    allowed = {"dimacs", "voxels", "barn"}
    if selectors - allowed:
        raise ValueError(f"unknown format cohorts: {sorted(selectors - allowed)}")
    inventory = []
    with runs_path.open("a", encoding="utf-8", buffering=1) as output_stream:
        if "dimacs" in selectors:
            inventory.extend(
                _run_dimacs_dataset(
                    root=root, index=index, output_stream=output_stream, completed=completed
                )
            )
        if "voxels" in selectors:
            inventory.extend(
                _run_voxel_datasets(
                    root=root, index=index, output_stream=output_stream, completed=completed
                )
            )
        if "barn" in selectors:
            inventory.extend(
                _run_barn_dataset(
                    root=root, index=index, output_stream=output_stream, completed=completed
                )
            )
    _write_json = {
        "schema_version": "pathplanning_format_cohort_inventory_v1",
        "dataset_index_sha256": _sha_file(index_path),
        "measurement_count": sum(
            1 for row in _read_jsonl(runs_path) if row.get("schema_version")
        ),
        "inventory": inventory,
        "unsupported": {
            "movingai_terrain": "source_terrain_cost_table_missing",
            "omplapp_configs": "omplapp_robot_mesh_collision_checker_required; OMPL runtime not bundled",
            "temporal_and_multi_agent": "installed_sources_are_static_single_agent",
            "kinodynamic_and_vehicle_planners": "dataset_robot_and_control_models_not_available",
            "jit_star": "BARN_point_xy_cohort_has_no_robot_jacobian_for_manipulability_scoring",
        },
    }
    (campaign / "inventory.json").write_text(
        json.dumps(_write_json, sort_keys=True, indent=2) + "\n", encoding="utf-8"
    )
    counts = Counter(row.get("outcome", {}).get("status", row.get("status", "unknown")) for row in _read_jsonl(runs_path))
    summary = {
        "schema_version": "pathplanning_format_cohort_summary_v1",
        "dataset_index_sha256": _write_json["dataset_index_sha256"],
        "measurements": _write_json["measurement_count"],
        "status_counts": dict(sorted(counts.items())),
        "inventory_records": len(inventory),
        "input_errors": sum(row.get("status") == "input_error" for row in inventory),
        "unsupported": _write_json["unsupported"],
    }
    (campaign / "summary.json").write_text(
        json.dumps(summary, sort_keys=True, indent=2) + "\n", encoding="utf-8"
    )
    return summary


def main(argv: list[str] | None = None) -> int:
    import sys

    if str(_ROOT) not in sys.path:
        sys.path.insert(0, str(_ROOT))
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    run = commands.add_parser("run")
    run.add_argument("--dataset-root", type=Path, default=Path("benchmark-results/datasets"))
    run.add_argument("--dataset-index", type=Path, default=Path("benchmark-results/datasets/index.json"))
    run.add_argument("--output", type=Path, required=True)
    run.add_argument("--sources", nargs="+", choices=("dimacs", "voxels", "barn"), default=("dimacs", "voxels", "barn"))
    args = parser.parse_args(argv)
    result = run_format_cohorts(
        args.dataset_root, args.dataset_index, args.output, sources=args.sources
    )
    print(json.dumps(result, sort_keys=True))
    return 0 if result["status_counts"].get("error", 0) == 0 and result["input_errors"] == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
