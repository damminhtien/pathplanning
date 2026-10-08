"""Run D* Lite and repair the route after blocking a directed edge."""

from __future__ import annotations

from math import inf

from pathplanning.native import NativeGraph
from pathplanning.planners.search.dstar_lite import DStarLitePlanner


def main() -> None:
    nodes = [(x, y) for y in range(4) for x in range(5)]
    edges = []
    for x, y in nodes:
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            next_x, next_y = x + dx, y + dy
            if 0 <= next_x < 5 and 0 <= next_y < 4:
                edges.append(((x, y), (next_x, next_y), 1.0))
    graph: NativeGraph[tuple[int, int]] = NativeGraph.from_edges(
        nodes,
        edges,
        heuristic=lambda first, second: abs(first[0] - second[0]) + abs(first[1] - second[1]),
    )
    start: tuple[int, int] = (0, 1)
    goal: tuple[int, int] = (4, 1)
    planner = DStarLitePlanner(graph, start, goal)
    try:
        initial = planner.plan()
        if initial.path is None:
            raise RuntimeError("D* Lite did not return the initial route")
        print(f"initial cost={initial.stats['path_cost']:.1f}, path={initial.path.tolist()}")
        planner.update_edges([((1, 1), (2, 1), inf), ((2, 1), (1, 1), inf)])
        repaired = planner.plan()
        if repaired.path is None:
            raise RuntimeError("D* Lite did not return the repaired route")
        print(f"repaired cost={repaired.stats['path_cost']:.1f}, path={repaired.path.tolist()}")
    finally:
        planner.close()


if __name__ == "__main__":
    main()
