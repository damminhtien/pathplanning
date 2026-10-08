"""Small EECBS example on a four-node cycle."""

from pathplanning import MultiAgentProblem, plan_multi_agent
from pathplanning.native import NativeGraph


def main() -> None:
    graph: NativeGraph[int] = NativeGraph.from_edges(
        nodes=(0, 1, 2, 3),
        edges=((0, 1, 1.0), (1, 2, 1.0), (2, 3, 1.0), (3, 0, 1.0)),
        directed=False,
    )
    result = plan_multi_agent(
        MultiAgentProblem(graph, starts=(0, 2), goals=(2, 0)),
        params={"w": 1.1},
    )
    print(f"paths={result.paths}, sum_of_costs={result.sum_of_costs}, makespan={result.makespan}")


if __name__ == "__main__":
    main()
