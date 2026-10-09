"""Strict MovingAI land-grid inputs and reproducible benchmark workloads."""

from __future__ import annotations

from dataclasses import dataclass, field
import hashlib
import math
from pathlib import Path
import re
from typing import TYPE_CHECKING, Mapping, Sequence

import numpy as np
from numpy.typing import NDArray

if TYPE_CHECKING:
    from pathplanning.native.graph import NativeGraph

MOVEMENT_PROFILE = "land_octile_v1"
MOTIONS: tuple[tuple[int, int], ...] = (
    (-1, 0),
    (-1, 1),
    (0, 1),
    (1, 1),
    (1, 0),
    (1, -1),
    (0, -1),
    (-1, -1),
)
_PASSABLE = frozenset(".GS")
_BLOCKED = frozenset("@OTW")
_NONNEGATIVE_INT = re.compile(r"(?:0|[1-9][0-9]*)\Z")


class DatasetError(ValueError):
    """A rejected dataset input, with a stable reason code for manifests."""

    def __init__(self, code: str, message: str) -> None:
        super().__init__(message)
        self.code = code


def _uint(text: str, label: str, *, positive: bool = False) -> int:
    if not _NONNEGATIVE_INT.fullmatch(text):
        raise DatasetError("invalid_field", f"{label} must be a non-negative integer: {text!r}")
    value = int(text)
    if positive and value == 0:
        raise DatasetError("invalid_field", f"{label} must be positive")
    return value


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


@dataclass(slots=True)
class MovingAIGrid:
    """Original ASCII occupancy with an on-demand, no-corner-cutting CSR view.

    ``occupancy[y, x]`` is true for a blocked cell. Every cell keeps its slot,
    including blocked cells, so native node IDs are always ``y * width + x``.
    """

    path: Path
    width: int
    height: int
    symbols: tuple[str, ...]
    occupancy: NDArray[np.bool_]
    map_sha256: str
    _csr: tuple[NDArray[np.uint64], NDArray[np.uint64], NDArray[np.float64]] | None = field(
        default=None, init=False, repr=False
    )

    @classmethod
    def from_file(cls, path: str | Path) -> MovingAIGrid:
        return load_map(path)

    @property
    def x_range(self) -> int:
        return self.width

    @property
    def y_range(self) -> int:
        return self.height

    @property
    def node_count(self) -> int:
        return self.width * self.height

    @property
    def free_count(self) -> int:
        return self.node_count - int(np.count_nonzero(self.occupancy))

    @property
    def edge_count(self) -> int:
        return int(self.to_csr()[0][-1])

    def id_for(self, cell: tuple[int, int]) -> int:
        x_coord, y_coord = cell
        if not (0 <= x_coord < self.width and 0 <= y_coord < self.height):
            raise DatasetError("out_of_bounds", f"cell {cell!r} is outside {self.path}")
        return y_coord * self.width + x_coord

    def cell_for(self, node_id: int) -> tuple[int, int]:
        if not isinstance(node_id, int) or not 0 <= node_id < self.node_count:
            raise DatasetError("out_of_bounds", f"node ID {node_id!r} is outside {self.path}")
        return node_id % self.width, node_id // self.width

    def is_passable(self, node_id: int) -> bool:
        x_coord, y_coord = self.cell_for(node_id)
        return not bool(self.occupancy[y_coord, x_coord])

    def neighbors(self, node_id: int):
        """Expose the same legal edges for the public discrete-graph protocol."""
        if not self.is_passable(node_id):
            return
        x_coord, y_coord = self.cell_for(node_id)
        for dx, dy in MOTIONS:
            next_x, next_y = x_coord + dx, y_coord + dy
            if not (0 <= next_x < self.width and 0 <= next_y < self.height):
                continue
            if self.occupancy[next_y, next_x]:
                continue
            if dx and dy and (self.occupancy[y_coord, next_x] or self.occupancy[next_y, x_coord]):
                continue
            yield next_y * self.width + next_x

    def edge_cost(self, source: int, target: int) -> float:
        """Return the profile cost or infinity for an illegal edge."""
        if not self.is_passable(source) or not self.is_passable(target):
            return math.inf
        x0, y0 = self.cell_for(source)
        x1, y1 = self.cell_for(target)
        dx, dy = abs(x1 - x0), abs(y1 - y0)
        if max(dx, dy) != 1:
            return math.inf
        if dx and dy and (self.occupancy[y0, x1] or self.occupancy[y1, x0]):
            return math.inf
        return math.sqrt(2.0) if dx and dy else 1.0

    def heuristic(self, node_id: int, goal_id: int) -> float:
        x0, y0 = self.cell_for(node_id)
        x1, y1 = self.cell_for(goal_id)
        dx, dy = abs(x1 - x0), abs(y1 - y0)
        return max(dx, dy) + (math.sqrt(2.0) - 1.0) * min(dx, dy)

    def _motion_mask(self, dx: int, dy: int) -> NDArray[np.bool_]:
        """Vectorized valid source cells for one motion, using raw occupancy."""
        x_lo, x_hi = max(0, -dx), min(self.width, self.width - dx)
        y_lo, y_hi = max(0, -dy), min(self.height, self.height - dy)
        valid = np.zeros((self.height, self.width), dtype=np.bool_)
        if x_lo >= x_hi or y_lo >= y_hi:
            return valid
        walkable = ~self.occupancy
        area = (
            walkable[y_lo:y_hi, x_lo:x_hi] & walkable[y_lo + dy : y_hi + dy, x_lo + dx : x_hi + dx]
        )
        if dx and dy:
            area &= walkable[y_lo:y_hi, x_lo + dx : x_hi + dx]
            area &= walkable[y_lo + dy : y_hi + dy, x_lo:x_hi]
        valid[y_lo:y_hi, x_lo:x_hi] = area
        return valid

    def to_csr(
        self,
    ) -> tuple[NDArray[np.uint64], NDArray[np.uint64], NDArray[np.float64]]:
        """Build directed CSR in the repository's fixed eight-motion order."""
        if self._csr is not None:
            return self._csr
        masks = [self._motion_mask(dx, dy) for dx, dy in MOTIONS]
        degrees = np.zeros(self.node_count, dtype=np.uint8)
        for mask in masks:
            degrees += mask.reshape(-1)
        offsets = np.empty(self.node_count + 1, dtype=np.uint64)
        offsets[0] = 0
        np.cumsum(degrees, out=offsets[1:])
        edge_count = int(offsets[-1])
        indices = np.empty(edge_count, dtype=np.uint64)
        costs = np.empty(edge_count, dtype=np.float64)
        next_slot = offsets[:-1].copy()
        for (dx, dy), mask in zip(MOTIONS, masks):
            sources = np.flatnonzero(mask).astype(np.uint64)
            positions = next_slot[sources]
            indices[positions] = sources.astype(np.int64) + dy * self.width + dx
            costs[positions] = math.sqrt(2.0) if dx and dy else 1.0
            next_slot[sources] += 1
        offsets.flags.writeable = False
        indices.flags.writeable = False
        costs.flags.writeable = False
        self._csr = offsets, indices, costs
        return self._csr

    def to_native_graph(self) -> NativeGraph[int]:
        """Copy the profile's CSR into a fresh, independently owned native graph."""
        from pathplanning.native import NativeGraph

        return NativeGraph[int].from_csr(*self.to_csr())

    def native_heuristic_values(self, goal_id: int) -> NDArray[np.float64]:
        """Precompute admissible octile values for all node slots, including walls."""
        goal_x, goal_y = self.cell_for(goal_id)
        if not self.is_passable(goal_id):
            raise DatasetError("blocked_endpoint", "goal is blocked")
        x_coords = np.tile(np.arange(self.width, dtype=np.float64), self.height)
        y_coords = np.repeat(np.arange(self.height, dtype=np.float64), self.width)
        dx = np.abs(x_coords - goal_x)
        dy = np.abs(y_coords - goal_y)
        return np.ascontiguousarray(
            np.maximum(dx, dy) + (math.sqrt(2.0) - 1.0) * np.minimum(dx, dy)
        )


def load_map(path: str | Path) -> MovingAIGrid:
    """Read an octile ``.map`` file, rejecting unknown or malformed content."""
    map_path = Path(path).resolve(strict=True)
    try:
        lines = map_path.read_text(encoding="ascii").splitlines()
    except UnicodeError as exc:
        raise DatasetError("invalid_encoding", f"map is not ASCII: {map_path}") from exc
    if len(lines) < 5 or lines[0] != "type octile" or lines[3] != "map":
        raise DatasetError("invalid_header", f"invalid octile map header: {map_path}")
    height_fields, width_fields = lines[1].split(), lines[2].split()
    if len(height_fields) != 2 or height_fields[0] != "height":
        raise DatasetError("invalid_header", f"invalid height header: {map_path}")
    if len(width_fields) != 2 or width_fields[0] != "width":
        raise DatasetError("invalid_header", f"invalid width header: {map_path}")
    height = _uint(height_fields[1], "height", positive=True)
    width = _uint(width_fields[1], "width", positive=True)
    symbols = tuple(lines[4:])
    if len(symbols) != height or any(len(row) != width for row in symbols):
        raise DatasetError(
            "dimension_mismatch", f"map rows do not match {width}x{height}: {map_path}"
        )
    unknown = sorted(set("".join(symbols)) - _PASSABLE - _BLOCKED)
    if unknown:
        raise DatasetError("unknown_symbol", f"unknown map symbols {unknown!r}: {map_path}")
    occupancy = np.fromiter(
        (symbol in _BLOCKED for row in symbols for symbol in row),
        dtype=np.bool_,
        count=width * height,
    ).reshape(height, width)
    occupancy.flags.writeable = False
    return MovingAIGrid(map_path, width, height, symbols, occupancy, _sha256_file(map_path))


@dataclass(frozen=True, slots=True)
class MovingAIScenario:
    family: str
    scenario_path: Path
    dataset_root: Path
    line_number: int
    bucket: int
    map_name: str
    map_path: Path
    map: MovingAIGrid
    start_id: int
    goal_id: int
    optimal_length_str: str
    optimal_length: float


def _resolve_map(name: str, scenario_path: Path, root: Path) -> Path:
    relative = Path(name)
    if relative.is_absolute() or ".." in relative.parts or not name or "\\" in name:
        raise DatasetError("invalid_map_path", f"map path escapes dataset root: {name!r}")
    candidates = [(root / relative).resolve(), (scenario_path.parent / relative).resolve()]
    matching = [path for path in dict.fromkeys(candidates) if path.is_file()]
    if not matching:
        raise DatasetError("missing_map", f"map does not exist within dataset root: {name!r}")
    if len(matching) > 1:
        raise DatasetError("ambiguous_map", f"multiple maps match scenario entry: {name!r}")
    map_path = matching[0]
    if not map_path.is_relative_to(root):
        raise DatasetError("invalid_map_path", f"map path escapes dataset root: {name!r}")
    return map_path


def parse_scenario(
    path: str | Path,
    dataset_root: str | Path,
    *,
    family: str = "unknown",
) -> list[MovingAIScenario]:
    """Parse nine-field scenario rows and resolve each row's map inside root."""
    scenario_path = Path(path).resolve(strict=True)
    root = Path(dataset_root).resolve(strict=True)
    if not scenario_path.is_relative_to(root):
        raise DatasetError("invalid_scenario_path", f"scenario is outside dataset root: {path}")
    try:
        lines = scenario_path.read_text(encoding="ascii").splitlines()
    except UnicodeError as exc:
        raise DatasetError("invalid_encoding", f"scenario is not ASCII: {scenario_path}") from exc
    if not lines or lines[0].strip() not in {"version 1", "version 1.0"}:
        raise DatasetError("invalid_version", f"unsupported scenario header: {scenario_path}")
    maps: dict[Path, MovingAIGrid] = {}
    scenarios: list[MovingAIScenario] = []
    for line_number, line in enumerate(lines[1:], start=2):
        fields = line.split()
        if len(fields) != 9:
            raise DatasetError(
                "invalid_row", f"scenario line {line_number} must have nine fields: {scenario_path}"
            )
        bucket_text, map_name, width_text, height_text, sx, sy, gx, gy, optimum_text = fields
        bucket = _uint(bucket_text, "bucket")
        width = _uint(width_text, "scenario width", positive=True)
        height = _uint(height_text, "scenario height", positive=True)
        start = (_uint(sx, "start x"), _uint(sy, "start y"))
        goal = (_uint(gx, "goal x"), _uint(gy, "goal y"))
        try:
            optimum = float(optimum_text)
        except ValueError as exc:
            raise DatasetError("invalid_optimum", f"invalid optimum: {optimum_text!r}") from exc
        if not math.isfinite(optimum) or optimum < 0:
            raise DatasetError("invalid_optimum", f"invalid optimum: {optimum_text!r}")
        map_path = _resolve_map(map_name, scenario_path, root)
        if map_path not in maps:
            maps[map_path] = load_map(map_path)
        grid = maps[map_path]
        if (width, height) != (grid.width, grid.height):
            raise DatasetError(
                "unsupported_scaled_scenario",
                f"scenario line {line_number} scales {map_name!r}; profile requires original dimensions",
            )
        start_id = grid.id_for(start)
        goal_id = grid.id_for(goal)
        if not grid.is_passable(start_id) or not grid.is_passable(goal_id):
            raise DatasetError(
                "blocked_endpoint", f"scenario line {line_number} uses a blocked endpoint"
            )
        scenarios.append(
            MovingAIScenario(
                family,
                scenario_path,
                root,
                line_number,
                bucket,
                map_name,
                map_path,
                grid,
                start_id,
                goal_id,
                optimum_text,
                optimum,
            )
        )
    return scenarios


parse_scenarios = parse_scenario


def make_case_record(scenario: MovingAIScenario, reference_cost: float | None) -> dict[str, object]:
    """Create the runner's JSON-safe workload record with stable input identity."""
    from scripts.shortest_path_benchmark.contract import stable_id

    grid = scenario.map
    identity = {
        "map_sha256": grid.map_sha256,
        "movement_profile": MOVEMENT_PROFILE,
        "start": scenario.start_id,
        "goal": scenario.goal_id,
    }
    return {
        "workload_id": stable_id("workload", identity),
        "family": scenario.family,
        "map_path": scenario.map_path.relative_to(scenario.dataset_root).as_posix(),
        "map_sha256": grid.map_sha256,
        "scenario_path": scenario.scenario_path.relative_to(scenario.dataset_root).as_posix(),
        "scenario_line": scenario.line_number,
        "bucket": scenario.bucket,
        "start": scenario.start_id,
        "goal": scenario.goal_id,
        "scenario_optimum": scenario.optimal_length_str,
        "reference_cost": reference_cost,
        "node_slots": grid.node_count,
        "free_nodes": grid.free_count,
        "directed_edges": grid.edge_count,
        "movement_profile": MOVEMENT_PROFILE,
    }


@dataclass(frozen=True, slots=True)
class PilotSelection:
    work_cases: tuple[MovingAIScenario, ...]
    latency_cases: tuple[MovingAIScenario, ...]
    metadata: dict[str, object]


def _case_hash(scenario: MovingAIScenario, seed: int) -> str:
    relative_scenario = scenario.scenario_path.relative_to(scenario.dataset_root)
    identity = (
        f"{seed}|{scenario.map.map_sha256}|{scenario.start_id}|{scenario.goal_id}|"
        f"{relative_scenario.as_posix()}|{scenario.line_number}"
    )
    return hashlib.sha256(identity.encode("utf-8")).hexdigest()


def select_pilot_cases(
    scenarios_by_family: Mapping[str, Sequence[MovingAIScenario]],
    *,
    seed: int = 7,
    maps_per_family: int = 3,
    quantile_bins: int = 5,
    work_per_bin: int = 20,
    latency_per_bin: int = 4,
) -> PilotSelection:
    """Freeze map and query selection before observing candidate outcomes.

    Map ranks use passable cell count, then relative name. Cost bins are rank
    quantiles of source scenario optima; hash ordering selects rows within each
    bin. Repeated start/goal inputs are selected once per map.
    """
    if min(maps_per_family, quantile_bins, work_per_bin, latency_per_bin) <= 0:
        raise ValueError("pilot selection sizes must be positive")
    if latency_per_bin > work_per_bin:
        raise ValueError("latency subset cannot exceed work selection")
    work_cases: list[MovingAIScenario] = []
    latency_cases: list[MovingAIScenario] = []
    families: dict[str, object] = {}
    workload_bins: dict[str, int] = {}

    for family in sorted(scenarios_by_family):
        rows_by_map: dict[Path, list[MovingAIScenario]] = {}
        for scenario in scenarios_by_family[family]:
            if scenario.family != family:
                raise ValueError("scenario family does not match input group")
            rows_by_map.setdefault(scenario.map_path, []).append(scenario)
        ordered_maps = sorted(
            rows_by_map,
            key=lambda path: (
                rows_by_map[path][0].map.free_count,
                path.relative_to(rows_by_map[path][0].dataset_root).as_posix(),
            ),
        )
        if not ordered_maps:
            families[family] = {"selected_maps": [], "available_maps": 0, "bins": {}}
            continue
        positions = [0, (len(ordered_maps) - 1) // 2, len(ordered_maps) - 1]
        selected_maps = list(dict.fromkeys(ordered_maps[position] for position in positions))
        if maps_per_family != 3:
            evenly_spaced = [
                round(index * (len(ordered_maps) - 1) / max(1, maps_per_family - 1))
                for index in range(maps_per_family)
            ]
            selected_maps = list(
                dict.fromkeys(ordered_maps[position] for position in evenly_spaced)
            )
        map_metadata: dict[str, object] = {}
        for map_path in selected_maps:
            rows = rows_by_map[map_path]
            unique: dict[tuple[int, int], MovingAIScenario] = {}
            for scenario in sorted(rows, key=lambda row: (row.scenario_path, row.line_number)):
                unique.setdefault((scenario.start_id, scenario.goal_id), scenario)
            cost_order = sorted(
                unique.values(),
                key=lambda row: (row.optimal_length, _case_hash(row, seed)),
            )
            bins: list[list[MovingAIScenario]] = [[] for _ in range(quantile_bins)]
            for rank, scenario in enumerate(cost_order):
                bins[min(quantile_bins - 1, rank * quantile_bins // len(cost_order))].append(
                    scenario
                )
            bin_metadata: list[dict[str, object]] = []
            for bin_index, candidates in enumerate(bins):
                selected = sorted(candidates, key=lambda row: _case_hash(row, seed))[:work_per_bin]
                latency = selected[:latency_per_bin]
                work_cases.extend(selected)
                latency_cases.extend(latency)
                for scenario in selected:
                    workload_id = str(make_case_record(scenario, None)["workload_id"])
                    workload_bins.setdefault(workload_id, bin_index)
                bin_metadata.append(
                    {
                        "index": bin_index,
                        "available": len(candidates),
                        "work_selected": len(selected),
                        "latency_selected": len(latency),
                        "cost_min": min((row.optimal_length for row in candidates), default=None),
                        "cost_max": max((row.optimal_length for row in candidates), default=None),
                    }
                )
            map_metadata[map_path.relative_to(rows[0].dataset_root).as_posix()] = {
                "map_sha256": rows[0].map.map_sha256,
                "free_nodes": rows[0].map.free_count,
                "scenario_rows": len(rows),
                "duplicate_queries": len(rows) - len(unique),
                "bins": bin_metadata,
            }
        families[family] = {
            "available_maps": len(ordered_maps),
            "selected_maps": list(map_metadata),
            "maps": map_metadata,
        }

    metadata: dict[str, object] = {
        "selection_version": "pilot_rank_quantiles_v1",
        "workload_seed": seed,
        "maps_per_family": maps_per_family,
        "quantile_bins": quantile_bins,
        "work_per_bin": work_per_bin,
        "latency_per_bin": latency_per_bin,
        "families": families,
        "work_cases": len(work_cases),
        "latency_cases": len(latency_cases),
        "workload_bins": workload_bins,
    }
    return PilotSelection(tuple(work_cases), tuple(latency_cases), metadata)
