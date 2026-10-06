"""Independent shortest-path oracle and path validation on raw occupancy."""

from __future__ import annotations

from dataclasses import dataclass
from decimal import Decimal, InvalidOperation
import heapq
import math
from typing import Sequence

from scripts.shortest_path_benchmark.workloads import MOVEMENT_PROFILE, MovingAIGrid

REFERENCE_VERSION = "occupancy_dijkstra_v1"
PATH_ABS_TOL = 1e-8
PATH_REL_TOL = 1e-10
_SQRT2 = math.sqrt(2.0)
_MOVINGAI_SCENARIO_DIAGONAL = Decimal("1.414213562")
# Deliberately independent of the CSR builder's motion masks and order.
_CARDINAL = ((-1, 0), (0, -1), (1, 0), (0, 1))
_DIAGONAL = ((-1, -1), (1, -1), (1, 1), (-1, 1))
_MOVES = (*_CARDINAL, *_DIAGONAL)


@dataclass(frozen=True, slots=True)
class ReferenceResult:
    reachable: bool
    cost: float | None
    expanded: int
    path: tuple[int, ...]


@dataclass(frozen=True, slots=True)
class PathValidation:
    valid: bool
    cost: float | None
    steps: int | None
    reason: str | None
    declared_match: bool | None
    optimal_match: bool | None


def _check_endpoint(grid: MovingAIGrid, node_id: int) -> None:
    if not grid.is_passable(node_id):
        raise ValueError(f"endpoint {node_id} is blocked")


def _solve(grid: MovingAIGrid, start: int, goal: int) -> ReferenceResult:
    _check_endpoint(grid, start)
    _check_endpoint(grid, goal)
    width, height = grid.width, grid.height
    blocked = memoryview(grid.occupancy).cast("B")
    distances = [math.inf] * grid.node_count
    parents = [-1] * grid.node_count
    distances[start] = 0.0
    frontier: list[tuple[float, int]] = [(0.0, start)]
    expanded = 0

    while frontier:
        distance, node_id = heapq.heappop(frontier)
        if distance != distances[node_id]:
            continue
        expanded += 1
        if node_id == goal:
            reverse_path: list[int] = []
            cursor = goal
            while cursor != -1:
                reverse_path.append(cursor)
                cursor = parents[cursor]
            return ReferenceResult(True, distance, expanded, tuple(reversed(reverse_path)))

        x_coord, y_coord = node_id % width, node_id // width
        for dx, dy in _MOVES:
            next_x, next_y = x_coord + dx, y_coord + dy
            if not (0 <= next_x < width and 0 <= next_y < height):
                continue
            neighbor = next_y * width + next_x
            if blocked[neighbor]:
                continue
            if (
                dx
                and dy
                and (blocked[y_coord * width + next_x] or blocked[next_y * width + x_coord])
            ):
                continue
            next_distance = distance + (_SQRT2 if dx and dy else 1.0)
            if next_distance < distances[neighbor]:
                distances[neighbor] = next_distance
                parents[neighbor] = node_id
                heapq.heappush(frontier, (next_distance, neighbor))
    return ReferenceResult(False, None, expanded, ())


class ReferenceCache:
    """Cache only oracle outputs; never share native graph or candidate state."""

    def __init__(self) -> None:
        self._results: dict[tuple[str, str, int, int, str], ReferenceResult] = {}

    def solve(self, grid: MovingAIGrid, start: int, goal: int) -> ReferenceResult:
        key = (grid.map_sha256, MOVEMENT_PROFILE, start, goal, REFERENCE_VERSION)
        if key not in self._results:
            self._results[key] = _solve(grid, start, goal)
        return self._results[key]


def reference_distance(
    grid: MovingAIGrid,
    start: int,
    goal: int,
    cache: ReferenceCache | None = None,
) -> float | None:
    """Return the exact octile shortest-path cost, or None if unreachable."""
    return (cache.solve(grid, start, goal) if cache is not None else _solve(grid, start, goal)).cost


def _cost_matches(left: float, right: float) -> bool:
    return math.isclose(left, right, abs_tol=PATH_ABS_TOL, rel_tol=PATH_REL_TOL)


def validate_path(
    grid: MovingAIGrid,
    path: Sequence[int] | None,
    start: int,
    goal: int,
    *,
    declared_cost: float | None = None,
    oracle_cost: float | None = None,
) -> PathValidation:
    """Check a candidate path independently of its search graph and declared cost."""
    if not path:
        return PathValidation(False, None, None, "no_path", None, None)
    if path[0] != start or path[-1] != goal:
        return PathValidation(False, None, None, "endpoint_mismatch", None, None)
    try:
        if not grid.is_passable(start) or not grid.is_passable(goal):
            return PathValidation(False, None, None, "blocked_endpoint", None, None)
    except (TypeError, ValueError):
        return PathValidation(False, None, None, "invalid_endpoint", None, None)

    width, height = grid.width, grid.height
    blocked = grid.occupancy
    cardinal_steps = diagonal_steps = 0
    previous = start
    for node_id in path[1:]:
        if not isinstance(node_id, int) or not 0 <= node_id < width * height:
            return PathValidation(False, None, None, "out_of_bounds", None, None)
        x0, y0 = previous % width, previous // width
        x1, y1 = node_id % width, node_id // width
        dx, dy = abs(x1 - x0), abs(y1 - y0)
        if max(dx, dy) != 1 or blocked[y1, x1]:
            return PathValidation(False, None, None, "illegal_step", None, None)
        if dx and dy:
            if blocked[y0, x1] or blocked[y1, x0]:
                return PathValidation(False, None, None, "corner_cut", None, None)
            diagonal_steps += 1
        else:
            cardinal_steps += 1
        previous = node_id
    cost = cardinal_steps + _SQRT2 * diagonal_steps
    declared_match = (
        None
        if declared_cost is None
        else (math.isfinite(declared_cost) and _cost_matches(cost, declared_cost))
    )
    optimal_match = None if oracle_cost is None else _cost_matches(cost, oracle_cost)
    return PathValidation(True, cost, len(path) - 1, None, declared_match, optimal_match)


def scenario_tolerance(optimal_length_str: str, reference_cost: float) -> float:
    """Allow one unit of the last printed decimal place plus floating tolerance."""
    try:
        number = Decimal(optimal_length_str)
    except InvalidOperation as exc:
        raise ValueError(f"invalid scenario optimum: {optimal_length_str!r}") from exc
    if not number.is_finite() or number < 0:
        raise ValueError(f"invalid scenario optimum: {optimal_length_str!r}")
    exponent = number.as_tuple().exponent
    assert isinstance(exponent, int)
    printed_unit = float(Decimal(1).scaleb(exponent))
    return max(PATH_ABS_TOL, PATH_REL_TOL * abs(reference_cost), printed_unit)


def scenario_matches_reference(optimal_length_str: str, reference_cost: float | None) -> bool:
    """Compare MovingAI's serialized optimum with the exact grid oracle."""
    if reference_cost is None:
        return False
    tolerance = scenario_tolerance(optimal_length_str, reference_cost)
    recorded = Decimal(optimal_length_str)
    if abs(float(recorded) - reference_cost) <= tolerance:
        return True

    # MovingAI scenario optima use 1.414213562 for diagonals, while the grid
    # oracle uses sqrt(2). Recover the unique integer move decomposition from
    # the exact oracle cost so this serialization difference does not look like
    # a dataset error. Candidate paths still use the strict oracle tolerance.
    oracle_tolerance = max(PATH_ABS_TOL, PATH_REL_TOL * abs(reference_cost))
    max_steps = math.ceil(reference_cost)
    decompositions: list[tuple[int, int]] = []
    for diagonal_steps in range(max_steps + 1):
        cardinal_steps = round(reference_cost - diagonal_steps * _SQRT2)
        if cardinal_steps < 0 or cardinal_steps + diagonal_steps > max_steps:
            continue
        reconstructed = cardinal_steps + diagonal_steps * _SQRT2
        if abs(reconstructed - reference_cost) <= oracle_tolerance:
            decompositions.append((cardinal_steps, diagonal_steps))
            if len(decompositions) > 1:
                return False
    if not decompositions:
        return False

    cardinal_steps, diagonal_steps = decompositions[0]
    expected = Decimal(cardinal_steps) + (Decimal(diagonal_steps) * _MOVINGAI_SCENARIO_DIAGONAL)
    exponent = recorded.as_tuple().exponent
    assert isinstance(exponent, int)
    printed_unit = Decimal(1).scaleb(exponent)
    representation_tolerance = printed_unit / 2 + Decimal(str(oracle_tolerance))
    return abs(expected - recorded) <= representation_tolerance
