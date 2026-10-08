"""Bounded structural measurements on explicit graphs and implicit grids.

This module never runs a candidate planner. Continuous spaces have no finite
vertex count; a sampled roadmap must not be reported as the original space.
"""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass
import gzip
import hashlib
import itertools
import math
from pathlib import Path
from typing import Any, cast
import xml.etree.ElementTree as ET

import numpy as np


@dataclass(frozen=True)
class Limits:
    """Deterministic limits; missing measurements remain visible in reports."""

    max_cells: int = 8_000_000
    max_arcs: int = 4_000_000
    sample_size: int = 4096
    query_sample_size: int = 256
    seed: int = 7

    def __post_init__(self) -> None:
        if min(self.max_cells, self.max_arcs, self.sample_size, self.query_sample_size) < 1:
            raise ValueError("Profiling limits must be positive")


def measurement(value: Any, method: str = "exact", **details: Any) -> dict[str, Any]:
    """Attach evidence to a value instead of confusing unknown values with zero."""
    if value is None and "reason" not in details:
        raise ValueError("An unavailable measurement requires a reason")
    return {"value": value, "method": method, **details}


def unavailable(reason: str) -> dict[str, Any]:
    return measurement(None, "unavailable", reason=reason)


def distribution(values: np.ndarray) -> dict[str, Any]:
    """Summarize a finite population or a separately identified sample."""
    if not values.size:
        return {"count": 0, "min": None, "median": None, "p95": None, "max": None}
    return {
        "count": int(values.size),
        "min": float(np.min(values)),
        "median": float(np.median(values)),
        "p95": float(np.percentile(values, 95)),
        "max": float(np.max(values)),
        "mean": float(np.mean(values)),
    }


def content_hash(path: Path) -> str:
    """Hash bytes without reading an entire asset into memory."""
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def _grid_degrees(blocked: np.ndarray) -> np.ndarray:
    """Count 8/26-neighbor edges, requiring every diagonal side voxel free."""
    free = ~blocked
    degree = np.zeros(blocked.shape, dtype=np.uint8)
    for move in itertools.product((-1, 0, 1), repeat=blocked.ndim):
        if not any(move):
            continue
        source = tuple(
            slice(max(0, -delta), min(size, size - delta))
            for size, delta in zip(blocked.shape, move)
        )
        valid = free[source].copy()
        axes = [axis for axis, delta in enumerate(move) if delta]
        # Every nonempty subset includes the destination and the proper sides.
        for count in range(1, len(axes) + 1):
            for subset in itertools.combinations(axes, count):
                destination = tuple(
                    slice(
                        part.start + (move[axis] if axis in subset else 0),
                        part.stop + (move[axis] if axis in subset else 0),
                    )
                    for axis, part in enumerate(source)
                )
                valid &= free[destination]
        degree[source] += valid
    return degree


def profile_grid(blocked: np.ndarray, limits: Limits) -> dict[str, Any]:
    """Measure topology under strict octile/26-connected Euclidean movement."""
    from scipy import ndimage

    if blocked.ndim not in (2, 3) or blocked.dtype != np.bool_:
        raise ValueError("Expected a 2D or 3D boolean occupancy array")
    slots = int(blocked.size)
    free_count = slots - int(np.count_nonzero(blocked))
    metrics = {
        "node_slots": measurement(slots),
        "free_nodes": measurement(free_count),
        "obstacle_fraction": measurement(1 - free_count / slots),
    }
    if slots > limits.max_cells:
        metrics.update(
            {
                key: unavailable("cell_limit")
                for key in (
                    "directed_edges",
                    "degree",
                    "connected_components",
                    "largest_component_fraction",
                    "clearance_cells",
                    "dead_end_fraction",
                )
            }
        )
        return metrics
    degree = _grid_degrees(blocked)
    metrics["directed_edges"] = measurement(int(np.sum(degree, dtype=np.uint64)))
    metrics["degree"] = measurement(distribution(degree[~blocked]))
    metrics["dead_end_fraction"] = measurement(
        float(np.mean(degree[~blocked] == 1)) if free_count else None,
        **({} if free_count else {"reason": "no_free_nodes"}),
    )
    # Strict diagonals connect only nodes already connected by cardinal moves.
    labels, components = cast(tuple[np.ndarray, int], ndimage.label(~blocked))
    sizes = np.bincount(labels.ravel())[1:]
    metrics["connected_components"] = measurement(int(components))
    metrics["largest_component_fraction"] = (
        measurement(int(sizes.max()) / free_count) if free_count else unavailable("no_free_nodes")
    )
    del labels, sizes
    # Padding treats the outer boundary as blocked. This measures cell-center
    # distance, not an exact robot clearance or a bottleneck cut statistic.
    clearance = cast(np.ndarray, ndimage.distance_transform_edt(np.pad(~blocked, 1)))
    interior = clearance[tuple(slice(1, -1) for _ in blocked.shape)][~blocked]
    metrics["clearance_cells"] = measurement(distribution(interior))
    return metrics


def load_octile(path: Path, *, terrain: bool = False) -> tuple[np.ndarray, dict[str, Any]]:
    """Keep terrain labels separate from land-grid occupancy semantics."""
    with path.open(encoding="ascii") as stream:
        header = [stream.readline().strip() for _ in range(4)]
        if header[0] != "type octile" or header[3] != "map":
            raise ValueError("Invalid octile header")
        if not header[1].startswith("height ") or not header[2].startswith("width "):
            raise ValueError("Missing octile dimensions")
        height, width = int(header[1].split()[1]), int(header[2].split()[1])
        if min(height, width) < 1:
            raise ValueError("Dimensions must be positive")
        rows = [line.rstrip("\r\n") for line in stream]
    if len(rows) != height or any(len(row) != width for row in rows):
        raise ValueError("Map dimensions do not match payload")
    symbols = np.frombuffer("".join(rows).encode("ascii"), dtype="S1").reshape(height, width)
    allowed = set("ABCDEFGHIJKLMNOPQRSTUVWXYZ" if terrain else ".GS@OTW")
    if set("".join(rows)) - allowed:
        raise ValueError("Unknown map symbols")
    blocked = (
        np.zeros((height, width), dtype=np.bool_)
        if terrain
        else np.isin(symbols, np.array([b"@", b"O", b"T", b"W"]))
    )
    counts = Counter("".join(rows)) if terrain else None
    return blocked, {"dimensions": [width, height], "terrain_counts": counts}


def profile_voxels(path: Path, limits: Limits) -> tuple[dict[str, Any], dict[str, Any]]:
    """Reject oversize grids before allocating dense occupancy; support rev_voxel."""
    opener = gzip.open if path.suffix == ".gz" else open
    with opener(path, "rt", encoding="ascii") as stream:
        header = stream.readline().split()
        if len(header) != 4 or header[0] not in {"voxel", "rev_voxel"}:
            raise ValueError("Invalid voxel header")
        dimensions = tuple(int(value) for value in header[1:])
        if min(dimensions) < 1:
            raise ValueError("Voxel dimensions must be positive")
        slots = math.prod(dimensions)
        metadata = {"dimensions": list(dimensions), "encoding": header[0]}
        if slots > limits.max_cells:
            return {
                "node_slots": measurement(slots, "header"),
                **{
                    key: unavailable("cell_limit")
                    for key in (
                        "free_nodes",
                        "obstacle_fraction",
                        "degree",
                        "directed_edges",
                        "connected_components",
                        "largest_component_fraction",
                        "clearance_cells",
                    )
                },
            }, metadata
        blocked = np.full(tuple(reversed(dimensions)), header[0] == "rev_voxel", dtype=np.bool_)
        duplicate_rows = 0
        for line in stream:
            coords = tuple(int(value) for value in line.split())
            if len(coords) != 3 or any(not 0 <= x < size for x, size in zip(coords, dimensions)):
                raise ValueError("Invalid voxel coordinate")
            index = tuple(reversed(coords))
            if bool(blocked[index]) == (header[0] == "voxel"):
                duplicate_rows += 1
            blocked[index] = header[0] == "voxel"
    metrics = profile_grid(blocked, limits)
    metrics["duplicate_coordinate_rows"] = measurement(duplicate_rows)
    return metrics, metadata


def profile_dimacs(path: Path, limits: Limits) -> dict[str, Any]:
    """Preserve directed multigraph arcs, self loops and source integer costs."""
    from scipy import sparse
    from scipy.sparse.csgraph import connected_components

    opener = gzip.open if path.suffix == ".gz" else open
    with opener(path, "rt", encoding="ascii") as stream:
        for line in stream:
            fields = line.split()
            if fields and fields[0] == "p":
                if len(fields) != 4 or fields[1] != "sp":
                    raise ValueError("Invalid DIMACS problem header")
                nodes, arcs = int(fields[2]), int(fields[3])
                break
        else:
            raise ValueError("Missing DIMACS problem header")
        if nodes < 1 or arcs < 0:
            raise ValueError("Invalid DIMACS graph size")
        if nodes > limits.max_cells or arcs > limits.max_arcs:
            return {
                "vertices": measurement(nodes, "header"),
                "arcs": measurement(arcs, "header"),
                "topology": unavailable("graph_limit"),
            }
        sources = np.empty(arcs, dtype=np.int64)
        targets = np.empty(arcs, dtype=np.int64)
        costs = np.empty(arcs, dtype=np.int64)
        index = 0
        for line in stream:
            fields = line.split()
            if not fields or fields[0] == "c":
                continue
            if fields[0] != "a" or len(fields) != 4 or index >= arcs:
                raise ValueError("Invalid DIMACS arc record")
            source, target, cost = (int(value) for value in fields[1:])
            if not 1 <= source <= nodes or not 1 <= target <= nodes:
                raise ValueError("DIMACS endpoint out of range")
            sources[index], targets[index], costs[index] = source - 1, target - 1, cost
            index += 1
        if index != arcs:
            raise ValueError("DIMACS arc count mismatch")
    # Connectivity ignores costs and parallel-arc multiplicity, unlike degree.
    graph = sparse.coo_matrix(
        (np.ones(arcs, dtype=np.bool_), (sources, targets)), shape=(nodes, nodes)
    ).tocsr()
    weak_count, weak_labels = connected_components(graph, directed=True, connection="weak")
    strong_count, strong_labels = connected_components(graph, directed=True, connection="strong")
    out_degree = np.bincount(sources, minlength=nodes)
    in_degree = np.bincount(targets, minlength=nodes)
    return {
        "vertices": measurement(nodes),
        "arcs": measurement(arcs),
        "arcs_per_vertex": measurement(arcs / nodes),
        "out_degree": measurement(distribution(out_degree)),
        "in_degree": measurement(distribution(in_degree)),
        "weak_components": measurement(int(weak_count)),
        "strong_components": measurement(int(strong_count)),
        "largest_weak_component_fraction": measurement(int(np.bincount(weak_labels).max()) / nodes),
        "largest_strong_component_fraction": measurement(
            int(np.bincount(strong_labels).max()) / nodes
        ),
        "reciprocal_arc_fraction": measurement(
            graph.multiply(graph.T).nnz / graph.nnz if graph.nnz else None,
            **({} if graph.nnz else {"reason": "no_arcs"}),
        ),
        "self_loops": measurement(int(np.count_nonzero(sources == targets))),
        "parallel_arcs": measurement(arcs - graph.nnz),
        "edge_cost": measurement(distribution(costs)),
        "negative_cost_arcs": measurement(int(np.count_nonzero(costs < 0))),
        "zero_cost_arcs": measurement(int(np.count_nonzero(costs == 0))),
    }


def profile_scenarios(
    path: Path, dimensions: list[int], limits: Limits, *, terrain: bool = False
) -> dict[str, Any]:
    """Audit supplied 2D queries; recorded optima are source claims, not oracles."""
    pairs: Counter[tuple[int, ...]] = Counter()
    lengths: list[float] = []
    displacements: list[float] = []
    scaled = 0
    with path.open(encoding="ascii") as stream:
        if stream.readline().strip() not in {"version 1", "version 1.0"}:
            raise ValueError("Unsupported scenario version")
        for line in stream:
            fields = line.split()
            if not fields:
                continue
            if len(fields) != 9:
                raise ValueError("Invalid scenario row")
            width, height = int(fields[2]), int(fields[3])
            sx, sy, gx, gy = (int(value) for value in fields[4:8])
            if [width, height] != dimensions:
                scaled += 1
                continue
            if not (0 <= sx < width and 0 <= gx < width and 0 <= sy < height and 0 <= gy < height):
                raise ValueError("Scenario endpoint out of range")
            optimum = float(fields[8])
            if optimum < 0 or not math.isfinite(optimum):
                raise ValueError("Invalid recorded scenario optimum")
            pairs[(sx, sy, gx, gy)] += 1
            lengths.append(optimum)
            displacements.append(
                math.hypot(gx - sx, gy - sy) / max(1, math.hypot(width - 1, height - 1))
            )
    reverse_pairs = sum((gx, gy, sx, sy) in pairs for sx, sy, gx, gy in pairs)
    return {
        "rows": measurement(sum(pairs.values())),
        "unique_ordered_pairs": measurement(len(pairs)),
        "duplicates": measurement(sum(pairs.values()) - len(pairs)),
        "reverse_pair_count": measurement(reverse_pairs),
        "scaled_rows": measurement(scaled),
        "normalized_displacement": measurement(distribution(np.asarray(displacements))),
        "recorded_optimal_cost": (
            unavailable("source_terrain_cost_table_missing")
            if terrain
            else measurement(distribution(np.asarray(lengths)), "source_unverified")
        ),
        "reachability": unavailable("no_independent_oracle_in_characterization"),
    }


def profile_voxel_scenarios(path: Path, dimensions: list[int]) -> dict[str, Any]:
    """Audit 3D scenario v1/v2 without interpreting search-work ratios as topology."""
    pairs: Counter[tuple[int, ...]] = Counter()
    costs: list[float] = []
    displacements: list[float] = []
    detours: list[float] = []
    work_ratios: list[float] = []
    with path.open(encoding="ascii") as stream:
        version = stream.readline().strip()
        if version not in {"version 1", "version 2"}:
            raise ValueError("Unsupported voxel scenario version")
        map_name = stream.readline().strip()
        if not map_name.endswith(".3dmap"):
            raise ValueError("Missing voxel scenario map name")
        diagonal = max(1.0, math.sqrt(sum((size - 1) ** 2 for size in dimensions)))
        for line in stream:
            fields = line.split()
            if not fields:
                continue
            if len(fields) != (8 if version == "version 1" else 9):
                raise ValueError("Invalid voxel scenario row")
            pair = tuple(int(value) for value in fields[:6])
            if any(not 0 <= coordinate < size for coordinate, size in zip(pair, dimensions * 2)):
                raise ValueError("Voxel scenario endpoint out of range")
            values = [float(value) for value in fields[6:]]
            if any(value < 0 or not math.isfinite(value) for value in values):
                raise ValueError("Invalid voxel scenario metric")
            pairs[pair] += 1
            costs.append(values[0])
            detours.append(values[1])
            displacements.append(math.dist(pair[:3], pair[3:]) / diagonal)
            if len(values) == 3:
                work_ratios.append(values[2])
    return {
        "rows": measurement(sum(pairs.values())),
        "unique_ordered_pairs": measurement(len(pairs)),
        "duplicates": measurement(sum(pairs.values()) - len(pairs)),
        "reverse_pair_count": measurement(sum(pair[3:] + pair[:3] in pairs for pair in pairs)),
        "normalized_displacement": measurement(distribution(np.asarray(displacements))),
        "recorded_optimal_cost": measurement(distribution(np.asarray(costs)), "source_unverified"),
        "source_heuristic_ratio": measurement(
            distribution(np.asarray(detours)), "source_unverified"
        ),
        "source_search_work_ratio": (
            measurement(distribution(np.asarray(work_ratios)), "source_unverified")
            if work_ratios
            else unavailable("scenario_version_has_no_work_ratio")
        ),
        "reachability": unavailable("no_independent_oracle_in_characterization"),
    }


def circle_grid_states(points: np.ndarray, radius: float = 0.25) -> np.ndarray:
    """OMPL Circle Grid's periodic circles at integer XY coordinates."""
    return np.sum((points - np.floor(points + 0.5)) ** 2, axis=1) > radius**2


def hypercube_states(points: np.ndarray, width: float = 0.1) -> np.ndarray:
    """OMPL's ordered edge corridor, not all edges of the hypercube."""
    dimension = points.shape[1]
    valid = np.zeros(len(points), dtype=np.bool_)
    for axis in range(dimension):
        valid |= np.all(points[:, :axis] >= 1 - width, axis=1) & np.all(
            points[:, axis + 1 :] <= width, axis=1
        )
    return valid


def profile_continuous(valid: Any, bounds: np.ndarray, limits: Limits) -> dict[str, Any]:
    """Uniform volume estimate with Wilson CI, independent of planner samples."""
    rng = np.random.default_rng(limits.seed)
    points = rng.uniform(bounds[0], bounds[1], size=(limits.sample_size, bounds.shape[1]))
    mask = np.asarray(valid(points), dtype=np.bool_)
    if mask.shape != (limits.sample_size,):
        raise ValueError("State validity returned a malformed batch")
    n = limits.sample_size
    successes = int(np.count_nonzero(mask))
    fraction = successes / n
    z = 1.959963984540054
    centre = (fraction + z * z / (2 * n)) / (1 + z * z / n)
    half = z * math.sqrt(fraction * (1 - fraction) / n + z * z / (4 * n * n)) / (1 + z * z / n)
    return {
        "vertices": unavailable("continuous_space"),
        "arcs": unavailable("continuous_space"),
        "free_volume_fraction": measurement(
            fraction,
            "uniform_sample",
            seed=limits.seed,
            sample_size=n,
            valid_samples=successes,
            ci95=[max(0, centre - half), min(1, centre + half)],
        ),
        "connected_components": unavailable("not_inferred_from_finite_samples"),
    }


def profile_barn(path: Path, limits: Limits) -> tuple[dict[str, Any], dict[str, Any]]:
    """Characterize BARN's cylinder geometry as a point robot in bounded XY."""
    root = ET.parse(path).getroot()
    circles: list[tuple[float, float, float]] = []

    def planar_pose(element: ET.Element) -> tuple[float, float, float]:
        pose = [float(value) for value in (element.findtext("pose") or "0 0 0 0 0 0").split()]
        if len(pose) != 6 or not all(math.isfinite(value) for value in pose) or any(pose[3:5]):
            raise ValueError("Unsupported tilted or invalid BARN pose")
        return pose[0], pose[1], pose[5]

    def compose(parent: tuple, child: tuple) -> tuple[float, float, float]:
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
                if radius_text is not None:
                    pose = compose(
                        compose(planar_pose(model), planar_pose(link)), planar_pose(collision)
                    )
                    radius = float(radius_text)
                    if not math.isfinite(radius) or radius <= 0:
                        raise ValueError("Invalid BARN cylinder radius")
                    circles.append((pose[0], pose[1], radius))
                elif collision.find("geometry/plane") is None:
                    raise ValueError("Unsupported BARN collision geometry")
    if not circles:
        raise ValueError("No BARN obstacle cylinders found")
    geometry = np.asarray(circles)
    # Describe the observed obstacle envelope explicitly; do not invent the
    # simulator's navigation bounds or the original Jackal's configuration space.
    bounds = np.vstack(
        (
            np.min(geometry[:, :2] - geometry[:, 2, None], axis=0),
            np.max(geometry[:, :2] + geometry[:, 2, None], axis=0),
        )
    )

    def valid(points: np.ndarray) -> np.ndarray:
        mask = np.ones(len(points), dtype=np.bool_)
        for x, y, radius in circles:
            mask &= np.sum((points - (x, y)) ** 2, axis=1) > radius**2
        return mask

    metrics = profile_continuous(valid, bounds, limits)
    metrics["obstacle_count"] = measurement(len(circles))
    return metrics, {
        "bounds": bounds.tolist(),
        "bounds_policy": "obstacle_envelope",
        "robot_model": "point_xy",
        "footprint_inflation": 0.0,
        "source_navigation_scores_comparable": False,
    }
