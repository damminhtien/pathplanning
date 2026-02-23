"""User-defined weighted grid world example for discrete planning."""

from __future__ import annotations

from collections.abc import Iterable
from dataclasses import dataclass, field
import math

from pathplanning.api import plan_discrete
from pathplanning.core.contracts import DiscreteProblem

GridCell = tuple[int, int]


@dataclass(slots=True)
class WeightedGridGraph:
    """Custom ``DiscreteGraph`` implementation with weighted terrain."""

    width: int
    height: int
    blocked: set[GridCell] = field(default_factory=set)
    terrain_weight: dict[GridCell, float] = field(default_factory=dict)
    diagonal: bool = True

    def __post_init__(self) -> None:
        if self.width <= 0 or self.height <= 0:
            raise ValueError("width and height must be > 0")
        self.blocked = set(self.blocked)
        self.terrain_weight = {cell: float(weight) for cell, weight in self.terrain_weight.items()}
        for cell, weight in self.terrain_weight.items():
            if weight <= 0.0:
                raise ValueError(f"terrain weight for {cell} must be > 0")

    def is_valid_node(self, node: GridCell) -> bool:
        x_coord, y_coord = node
        in_bounds = 0 <= x_coord < self.width and 0 <= y_coord < self.height
        return in_bounds and node not in self.blocked

    def neighbors(self, node: GridCell) -> Iterable[GridCell]:
        x_coord, y_coord = node
        motions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        if self.diagonal:
            motions.extend([(-1, -1), (-1, 1), (1, -1), (1, 1)])

        for dx, dy in motions:
            nxt = (x_coord + dx, y_coord + dy)
            if self.is_valid_node(nxt):
                yield nxt

    def edge_cost(self, a: GridCell, b: GridCell) -> float:
        if not self.is_valid_node(a) or not self.is_valid_node(b):
            return float("inf")
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])
        if dx > 1 or dy > 1 or (dx == 0 and dy == 0):
            return float("inf")
        base = math.sqrt(2.0) if dx == 1 and dy == 1 else 1.0
        weight = self.terrain_weight.get(b, 1.0)
        return base * weight

    def heuristic(self, node: GridCell, goal: GridCell) -> float:
        return math.hypot(goal[0] - node[0], goal[1] - node[1])


def build_custom_grid_problem() -> DiscreteProblem[GridCell]:
    """Create a weighted grid with walls and slow terrain."""
    blocked: set[GridCell] = set()

    # Wall with a narrow gap.
    for y_coord in range(1, 14):
        if y_coord == 7:
            continue
        blocked.add((10, y_coord))

    # A few scattered obstacles.
    blocked.update({(5, 5), (5, 6), (6, 6), (14, 3), (14, 4), (14, 5)})

    # Slow terrain corridor (still traversable).
    terrain_weight = {(x_coord, 9): 2.5 for x_coord in range(2, 17)}

    graph = WeightedGridGraph(
        width=20,
        height=16,
        blocked=blocked,
        terrain_weight=terrain_weight,
        diagonal=True,
    )
    return DiscreteProblem(graph=graph, start=(1, 1), goal=(18, 13))


if __name__ == "__main__":
    problem = build_custom_grid_problem()
    result = plan_discrete(problem, planner="astar")

    print(f"success={result.success}")
    print(f"stop_reason={result.stop_reason.value}")
    print(f"nodes={result.nodes}, iters={result.iters}")
    if result.path is not None:
        print(f"path_points={len(result.path)}")
