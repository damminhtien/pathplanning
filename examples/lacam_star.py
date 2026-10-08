"""Plan and improve a tiny multi-agent solution with LaCAM*."""

from pathplanning import MultiAgentProblem, plan_multi_agent
from pathplanning.native import NativeGraph

graph = NativeGraph.from_edges(
    nodes=(0, 1, 2, 3),
    edges=((0, 1, 1.0), (1, 2, 1.0), (2, 3, 1.0), (3, 0, 1.0)),
    directed=False,
)
problem = MultiAgentProblem(graph, starts=(0, 2), goals=(2, 0))
result = plan_multi_agent(problem, planner="lacam_star", seed=7)

print("success:", result.success)
print("paths:", result.paths)
print("sum of costs:", result.sum_of_costs)
print("makespan:", result.makespan)
