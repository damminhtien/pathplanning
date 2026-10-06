"""Deterministic map and query generators for shortest-path scaling sweeps.

Generation is lazy: callers can stream one map at a time and retain the exact
occupancy hash, requested parameters, observed density, and query coordinates.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import hashlib
import json
import math
import statistics
from typing import Iterator

import numpy as np

DISPLACEMENT_BINS = (0.0, 0.2, 0.4, 0.6, 0.8, 1.0)
SIZE_LEVELS = (64, 128, 256, 512, 1024)
DENSITY_LEVELS = (0.10, 0.20, 0.30, 0.40)
OPENING_WIDTHS = (1, 2, 4, 8)
HEURISTIC_ALPHAS = (0.0, 0.25, 0.5, 0.75, 1.0)


def _seed(*parts: object) -> int:
    encoded = json.dumps(parts, separators=(",", ":"), sort_keys=True).encode()
    return int.from_bytes(hashlib.sha256(encoded).digest()[:8], "big")


@dataclass(frozen=True)
class ScalingQuery:
    start: tuple[int, int]
    goal: tuple[int, int]
    displacement_bin: int


@dataclass(frozen=True)
class ScalingCase:
    sweep: str
    topology: str
    size: int
    seed: int
    requested_density: float | None
    opening_width: int | None
    blocked: np.ndarray
    queries: tuple[ScalingQuery, ...]
    connected_components: int
    largest_component_nodes: int

    def manifest_record(self) -> dict[str, object]:
        free_nodes = int(np.count_nonzero(~self.blocked))
        map_hash = hashlib.sha256(np.ascontiguousarray(self.blocked).tobytes()).hexdigest()
        identity = (
            self.sweep,
            self.topology,
            self.size,
            self.seed,
            self.requested_density,
            self.opening_width,
            map_hash,
        )
        case_id = hashlib.sha256(json.dumps(identity, separators=(",", ":")).encode()).hexdigest()
        return {
            "case_id": case_id,
            "sweep": self.sweep,
            "topology": self.topology,
            "width": self.size,
            "height": self.size,
            "seed": self.seed,
            "requested_density": self.requested_density,
            "observed_density": 1 - free_nodes / (self.size * self.size),
            "opening_width": self.opening_width,
            "map_sha256": map_hash,
            "node_slots": self.size * self.size,
            "free_nodes": free_nodes,
            "connected_components_4": self.connected_components,
            "largest_component_nodes_4": self.largest_component_nodes,
            "queries": [
                {
                    "start": list(query.start),
                    "goal": list(query.goal),
                    "displacement_bin": query.displacement_bin,
                }
                for query in self.queries
            ],
            "displacement_bin_edges": list(DISPLACEMENT_BINS),
        }


def connectivity_summary(blocked: np.ndarray) -> tuple[int, int]:
    """Count 4-neighbor free-space components for map covariates."""
    if blocked.ndim != 2 or blocked.dtype != np.bool_:
        raise ValueError("blocked must be a 2D bool array")
    height, width = blocked.shape
    visited = np.zeros_like(blocked)
    count = 0
    largest = 0
    for y, x in np.argwhere(~blocked):
        if visited[y, x]:
            continue
        count += 1
        size = 0
        queue = deque([(int(y), int(x))])
        visited[y, x] = True
        while queue:
            cy, cx = queue.popleft()
            size += 1
            for ny, nx in ((cy - 1, cx), (cy + 1, cx), (cy, cx - 1), (cy, cx + 1)):
                if (
                    0 <= ny < height
                    and 0 <= nx < width
                    and not blocked[ny, nx]
                    and not visited[ny, nx]
                ):
                    visited[ny, nx] = True
                    queue.append((ny, nx))
        largest = max(largest, size)
    return count, largest


def sample_queries(
    blocked: np.ndarray,
    *,
    seed: int,
    per_bin: int = 4,
    bin_edges: tuple[float, ...] = DISPLACEMENT_BINS,
) -> tuple[ScalingQuery, ...]:
    """Sample distinct passable pairs in fixed normalized displacement bins.

    Reachability is deliberately not filtered: disconnected inputs remain in
    the campaign for independent reference classification.
    """
    if blocked.ndim != 2 or blocked.dtype != np.bool_:
        raise ValueError("blocked must be a 2D bool array")
    if per_bin < 1 or len(bin_edges) < 2 or bin_edges[0] != 0 or bin_edges[-1] != 1:
        raise ValueError("Invalid displacement bins")
    if any(left >= right for left, right in zip(bin_edges, bin_edges[1:])):
        raise ValueError("Displacement bins must increase")
    free = np.argwhere(~blocked)
    if len(free) < 2:
        raise ValueError("At least two free cells are required")
    rng = np.random.default_rng(seed)
    maximum = math.hypot(blocked.shape[0] - 1, blocked.shape[1] - 1)
    result: list[ScalingQuery] = []
    used: set[tuple[int, int, int, int]] = set()
    for bin_index, (lower, upper) in enumerate(zip(bin_edges, bin_edges[1:])):
        attempts = 0
        while sum(query.displacement_bin == bin_index for query in result) < per_bin:
            if attempts >= 1_000_000:
                raise ValueError(f"Cannot fill displacement bin {bin_index}")
            attempts += 4096
            first = free[rng.integers(len(free), size=4096)]
            second = free[rng.integers(len(free), size=4096)]
            distances = np.hypot(first[:, 0] - second[:, 0], first[:, 1] - second[:, 1]) / maximum
            eligible = (distances >= lower) & (
                distances < upper if upper < 1 else distances <= upper
            )
            for index in np.flatnonzero(eligible):
                sy, sx = map(int, first[index])
                gy, gx = map(int, second[index])
                if sx == gx and sy == gy:
                    continue
                identity = (sx, sy, gx, gy)
                if identity in used:
                    continue
                used.add(identity)
                result.append(ScalingQuery((sx, sy), (gx, gy), bin_index))
                if sum(query.displacement_bin == bin_index for query in result) == per_bin:
                    break
    return tuple(result)


def _random_grid(size: int, density: float, seed: int) -> np.ndarray:
    if size < 2 or not 0 <= density < 1:
        raise ValueError("size must be >=2 and density in [0,1)")
    return (
        np.random.default_rng(_seed("random", size, density, seed)).random((size, size)) < density
    )


def _room_grid(size: int, opening_width: int, seed: int) -> np.ndarray:
    blocked = np.zeros((size, size), dtype=np.bool_)
    step = max(16, opening_width * 4 + 4)
    walls = list(range(step, size - 1, step))
    rng = np.random.default_rng(_seed("room", size, opening_width, seed))
    for x in walls:
        blocked[:, x] = True
        for y0, y1 in zip((0, *walls), (*walls, size)):
            if y1 - y0 < opening_width + 2:
                continue
            door = int(rng.integers(y0 + 1, y1 - opening_width + 1))
            blocked[door : door + opening_width, x] = False
    for y in walls:
        blocked[y, :] = True
        for x0, x1 in zip((0, *walls), (*walls, size)):
            if x1 - x0 < opening_width + 2:
                continue
            door = int(rng.integers(x0 + 1, x1 - opening_width + 1))
            blocked[y, door : door + opening_width] = False
    return blocked


def _serpentine_grid(size: int, opening_width: int, seed: int) -> np.ndarray:
    blocked = np.zeros((size, size), dtype=np.bool_)
    step = max(10, opening_width * 4 + 4)
    rng = np.random.default_rng(_seed("serpentine_maze", size, opening_width, seed))
    offset = int(rng.integers(0, max(1, step // 3)))
    first_left = bool(rng.integers(0, 2))
    for index, y in enumerate(range(step + offset, size - 1, step)):
        blocked[y, :] = True
        left = (index % 2 == 0) == first_left
        opening = slice(0, opening_width) if left else slice(size - opening_width, size)
        blocked[y, opening] = False
    return blocked


def _make_case(
    sweep: str,
    topology: str,
    size: int,
    seed: int,
    blocked: np.ndarray,
    *,
    requested_density: float | None = None,
    opening_width: int | None = None,
    queries_per_bin: int = 4,
) -> ScalingCase:
    count, largest = connectivity_summary(blocked)
    queries = sample_queries(
        blocked,
        seed=_seed("query", size, requested_density, opening_width, seed),
        per_bin=queries_per_bin,
    )
    return ScalingCase(
        sweep,
        topology,
        size,
        seed,
        requested_density,
        opening_width,
        blocked,
        queries,
        count,
        largest,
    )


def size_sweep(
    *,
    sizes: tuple[int, ...] = SIZE_LEVELS,
    seeds: tuple[int, ...] = (0, 1, 2, 3, 4),
    density: float = 0.20,
    queries_per_bin: int = 4,
) -> Iterator[ScalingCase]:
    for size in sizes:
        for seed in seeds:
            blocked = _random_grid(size, density, seed)
            yield _make_case(
                "size",
                "random",
                size,
                seed,
                blocked,
                requested_density=density,
                queries_per_bin=queries_per_bin,
            )


def density_sweep(
    *,
    size: int = 512,
    densities: tuple[float, ...] = DENSITY_LEVELS,
    seeds: tuple[int, ...] = (0, 1, 2, 3, 4),
    queries_per_bin: int = 4,
) -> Iterator[ScalingCase]:
    for density in densities:
        for seed in seeds:
            blocked = _random_grid(size, density, seed)
            yield _make_case(
                "density",
                "random",
                size,
                seed,
                blocked,
                requested_density=density,
                queries_per_bin=queries_per_bin,
            )


def topology_sweep(
    *,
    size: int = 512,
    opening_widths: tuple[int, ...] = OPENING_WIDTHS,
    seeds: tuple[int, ...] = (0, 1, 2, 3, 4),
    queries_per_bin: int = 4,
) -> Iterator[ScalingCase]:
    for topology in ("room", "serpentine_maze"):
        for width in opening_widths:
            if width < 1 or width >= size:
                raise ValueError("opening width must be in [1, size)")
            for seed in seeds:
                blocked = (
                    _room_grid(size, width, seed)
                    if topology == "room"
                    else _serpentine_grid(size, width, seed)
                )
                yield _make_case(
                    "topology",
                    topology,
                    size,
                    seed,
                    blocked,
                    opening_width=width,
                    queries_per_bin=queries_per_bin,
                )


def heuristic_variants(
    alphas: tuple[float, ...] = HEURISTIC_ALPHAS,
) -> tuple[dict[str, float | str], ...]:
    """List consistent h=alpha*octile variants on the same map/query cohort."""
    if any(not math.isfinite(alpha) or not 0 <= alpha <= 1 for alpha in alphas):
        raise ValueError("Heuristic alpha must be finite and within [0,1]")
    return tuple(
        {"variant_id": f"astar_halpha_{alpha:g}", "heuristic": "octile", "alpha": alpha}
        for alpha in alphas
    )


def octile_heuristic_array(size: int, goal: tuple[int, int], alpha: float = 1.0) -> np.ndarray:
    if size < 1 or not 0 <= goal[0] < size or not 0 <= goal[1] < size:
        raise ValueError("Goal is outside grid")
    if not math.isfinite(alpha) or not 0 <= alpha <= 1:
        raise ValueError("alpha must be within [0,1]")
    y, x = np.indices((size, size), dtype=np.int32)
    dx = np.abs(x - goal[0])
    dy = np.abs(y - goal[1])
    shorter = np.minimum(dx, dy)
    return np.ascontiguousarray(
        (dx + dy + (math.sqrt(2) - 2) * shorter).ravel() * alpha, dtype=np.float64
    )


def fit_loglog_slope(
    points: list[tuple[float, float]],
    *,
    draws: int = 2000,
    seed: int = 7,
) -> dict[str, object]:
    """Fit an empirical slope and bootstrap its CI; this is not a Big-O proof."""
    if (
        draws < 1
        or len(points) < 3
        or any(x <= 0 or y <= 0 or not math.isfinite(x + y) for x, y in points)
    ):
        raise ValueError("Need at least three finite positive x/y points and draws >=1")
    logs = [(math.log(x), math.log(y)) for x, y in points]

    def slope(samples: list[tuple[float, float]]) -> float | None:
        x_mean = statistics.mean(x for x, _ in samples)
        y_mean = statistics.mean(y for _, y in samples)
        denominator = sum((x - x_mean) ** 2 for x, _ in samples)
        return (
            sum((x - x_mean) * (y - y_mean) for x, y in samples) / denominator
            if denominator > 0
            else None
        )

    estimate = slope(logs)
    if estimate is None:
        raise ValueError("x values must vary")
    rng = np.random.default_rng(seed)
    bootstrapped = []
    for _ in range(draws):
        sampled = [logs[int(index)] for index in rng.integers(len(logs), size=len(logs))]
        value = slope(sampled)
        if value is not None:
            bootstrapped.append(value)
    return {
        "slope": estimate,
        "ci95": [float(np.percentile(bootstrapped, 2.5)), float(np.percentile(bootstrapped, 97.5))]
        if bootstrapped
        else None,
        "fit_x_range": [min(x for x, _ in points), max(x for x, _ in points)],
        "point_count": len(points),
        "bootstrap_draws": draws,
        "effective_bootstrap_draws": len(bootstrapped),
        "interpretation": "empirical_trend_only",
    }


def ablation_factor(
    base: dict[str, object],
    candidate: dict[str, object],
    *,
    factor: str,
) -> dict[str, object]:
    """Reject a proposed ablation when more than the named factor changes."""
    if factor not in base or factor not in candidate:
        raise ValueError(f"Missing ablation factor: {factor}")
    changed = {key for key in base.keys() | candidate.keys() if base.get(key) != candidate.get(key)}
    if changed != {factor}:
        raise ValueError(f"Expected only {factor} to change, got {sorted(changed)}")
    return {"factor": factor, "base": base[factor], "candidate": candidate[factor]}


__all__ = [
    "ScalingCase",
    "ScalingQuery",
    "ablation_factor",
    "connectivity_summary",
    "density_sweep",
    "fit_loglog_slope",
    "heuristic_variants",
    "octile_heuristic_array",
    "sample_queries",
    "size_sweep",
    "topology_sweep",
]
