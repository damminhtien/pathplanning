# LaCAM*

LaCAM* plans paths for multiple agents moving one edge per tick on a shared
undirected, unit-cost graph. It forbids vertex collisions and head-on edge swaps,
allows waiting, and keeps each agent at its goal after arrival. Its objective is
sum of costs; the result also reports makespan.

The native search lazily expands a constraint tree for each joint configuration.
It adds one agent's next-vertex constraint at a time, rejects partial assignments
that already conflict, and emits complete successor configurations only when
their leaves are visited. Candidate vertices are ordered by distance to each
agent's goal. The search keeps discovered configuration edges, repairs `g`
values and parent links with incremental Dijkstra updates, and continues after
the first collision-free solution to improve its cost. After finding a solution,
it uses seeded randomized extraction from the open set; the public `seed` keeps
that behavior reproducible. Exhausting the open set proves optimality. A time or
expansion budget can return a validated incumbent with the corresponding stop
reason.

The state space grows combinatorially with graph size and agent count. Set
`max_expansions` or `max_runtime_ms` for a bounded run on larger instances.
`max_materialized_nodes` applies when adapting a custom graph protocol to native
CSR; `NativeGraph` and `Grid2DMultiAgentAdapter` can be passed directly.

```python
from pathplanning import MultiAgentProblem, plan_multi_agent
from pathplanning.native import NativeGraph

graph = NativeGraph.from_edges(
    nodes=(0, 1, 2, 3),
    edges=((0, 1, 1.0), (1, 2, 1.0), (2, 3, 1.0), (3, 0, 1.0)),
    directed=False,
)
result = plan_multi_agent(
    MultiAgentProblem(graph, starts=(0, 2), goals=(2, 0)),
    planner="lacam_star",
    seed=7,
)
print(result.paths, result.sum_of_costs, result.makespan)
```

The implementation follows the lazy-constraint and anytime-search framework of
Engineering LaCAM* (AAMAS 2024). Its focused tests compare small instances with
an independent joint-state Dijkstra oracle; they do not establish asymptotic
claims.

References: [Engineering LaCAM* (AAMAS 2024)](https://aamas.csc.liv.ac.uk/Proceedings/aamas2024/pdfs/p1501.pdf),
[reference implementation](https://github.com/Kei18/lacam3).

Runnable example: [`examples/lacam_star.py`](../../examples/lacam_star.py).
Run from the repository root with `python -m examples.lacam_star`.
