"""Use bounded-suboptimal SIPP to trade a small cost bound for focal guidance."""

from __future__ import annotations

from pathplanning import TemporalProblem, plan_temporal
from pathplanning.native import NativeGraph


def main() -> None:
    graph: NativeGraph[int] = NativeGraph.from_edges(
        nodes=(0, 1, 2, 3, 4),
        edges=((0, 1, 1.0), (1, 2, 1.0), (2, 3, 1.0), (0, 4, 2.5), (4, 3, 1.0)),
    )
    problem = TemporalProblem(graph=graph, start=0, goal=3)
    result = plan_temporal(problem, planner="bounded_suboptimal_sipp", params={"w": 1.5})
    if not result.success:
        raise RuntimeError(f"FocalSIPP did not find a route: {result.stop_reason.value}")
    print(f"states={result.states}, elapsed={result.stats['path_cost']:.1f}")


if __name__ == "__main__":
    main()
