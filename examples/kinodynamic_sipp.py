"""Plan through an edge blockage without waiting at a moving configuration."""

from __future__ import annotations

from pathplanning import TemporalProblem, plan_temporal
from pathplanning.native import NativeGraph


def main() -> None:
    start = ("A", 0.0)
    moving = ("B", 1.0)
    goal = ("C", 1.0)
    graph: NativeGraph[tuple[str, float]] = NativeGraph.from_edges(
        nodes=(start, moving, goal),
        edges=((start, moving, 1.0), (moving, goal, 1.0)),
    )
    problem = TemporalProblem(
        graph=graph,
        start=start,
        goal=goal,
        edge_blocked={(moving, goal): [(0, 5)]},
        node_velocities={start: 0.0, moving: 1.0, goal: 1.0},
        edge_distances={(start, moving): 0.5, (moving, goal): 1.0},
        max_speed=1.0,
        max_acceleration=1.0,
        max_deceleration=1.0,
    )
    result = plan_temporal(problem, planner="kinodynamic_sipp")
    if not result.success:
        raise RuntimeError(f"Kinodynamic SIPP did not find a route: {result.stop_reason.value}")
    print(f"configurations={result.states}, arrival ticks={result.times}")


if __name__ == "__main__":
    main()
