# EECBS

`eecbs` plans paths for multiple agents on a shared, undirected, unit-weight
graph. Each timestep lets an agent traverse one edge or wait. Paths may not
share a vertex at the same timestep or traverse an edge in opposite directions
at once; an agent stays at its goal after arrival. The objective is sum of
arrival times, with makespan reported separately.

The native C++ implementation uses conflict-based search at the high level,
explicit estimation search with an online cost-to-go estimate for selecting
constraint-tree nodes, and focal A* for each constrained single-agent path. The
weight `w` must be finite and at least 1. A result stopped by a budget can carry
a feasible incumbent; the suboptimality guarantee applies only when
`stop_reason` is `success`.

```python
from pathplanning import MultiAgentProblem, plan
from pathplanning.native import NativeGraph

graph = NativeGraph.from_edges(
    nodes=(0, 1, 2, 3),
    edges=((0, 1, 1.0), (1, 2, 1.0), (2, 3, 1.0), (3, 0, 1.0)),
    directed=False,
)
result = plan(MultiAgentProblem(graph, starts=(0, 2), goals=(2, 0)), params={"w": 1.1})
```

`result.paths` contains one vertex sequence per agent. `sum_of_costs` sums the
edge counts through each agent's first final arrival and `makespan` is the
maximum of those counts. The initial implementation uses earliest conflicts and
does not include optional CBS improvements such as symmetry reasoning or a WDG
heuristic.

For occupancy grids, `Grid2DMultiAgentAdapter` turns free cells into a
four-connected graph with unit edge costs, regardless of the grid's movement
settings. Blocked cells are omitted from the resulting graph.

Reference: Li, Ruml, and Koenig, “EECBS: A Bounded-Suboptimal Search for Multi-
Agent Path Finding,” AAAI 2021. The implementation follows the paper's EES
high-level selection and focal low-level bound; online estimates use a running
average of observed child cost and conflict-count errors. See the [AAAI paper](https://ojs.aaai.org/index.php/AAAI/article/view/17466)
and the [author's PDF](https://www.cs.unh.edu/~ruml/papers/eecbs-aaai21.pdf).
