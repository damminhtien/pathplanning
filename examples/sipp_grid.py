"""Plan around a time-blocked grid cell using SIPP."""

from __future__ import annotations

from pathplanning import TemporalProblem, plan_temporal
from pathplanning.native import NativeGraph


def main() -> None:
    start: tuple[int, int] = (0, 0)
    goal: tuple[int, int] = (1, 1)
    lower: tuple[int, int] = (0, 1)
    upper: tuple[int, int] = (1, 0)
    graph: NativeGraph[tuple[int, int]] = NativeGraph.from_edges(
        nodes=(start, goal, lower, upper),
        edges=(
            (start, upper, 1.0),
            (upper, goal, 1.0),
            (start, lower, 1.0),
            (lower, goal, 1.0),
            (start, goal, 6.0),
        ),
    )
    problem = TemporalProblem(
        graph=graph,
        start=start,
        goal=goal,
        node_blocked={upper: [(0.0, 3.0)]},
        edge_blocked={(lower, goal): [(0.0, 3.0)]},
    )
    result = plan_temporal(problem)
    if not result.success:
        raise RuntimeError(f"SIPP did not find a route: {result.stop_reason.value}")
    print(
        f"states={result.states}, arrival times={result.times}, elapsed={result.stats['path_cost']:.1f}"
    )


if __name__ == "__main__":
    main()
