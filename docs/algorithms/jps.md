# Jump Point Search

`plan_discrete(problem, planner="jps")` runs the native C++ Jump Point Search
kernel on a uniform-cost 2D occupancy grid. It returns every traversed cell in
the path, even though the search expands only jump points.

The planner accepts a `Grid2DSearchSpace` with the standard eight motions and
the built-in cardinal/diagonal cost model (1 and √2). Diagonal moves cannot
pass between two blocked side cells. Exact cell goals are required. A grid
subclass that overrides `edge_cost` is rejected. The optional
`max_expansions` parameter limits expanded jump points; it must be a positive
integer. Seed and RNG are unused because the search is deterministic. Native
trace recording is supported. Result statistics include expanded jump points,
motion-validity checks, native kernel time, and path cost.

The implementation follows the JPS symmetry-pruning approach described by
Harabor and Grastien, with local forced-successor checks adapted to the strict
no-corner-cutting movement rule. The supported contract is limited to the
uniform 8-connected model for which standard JPS is optimal. Weighted terrain,
other movement costs, arbitrary graphs, and continuous motion are outside the
supported contract. The test suite compares returned costs with an independent
Dijkstra oracle over fixed and seeded-random occupancy maps; that empirical
coverage is not a formal proof of the strict-corner adaptation.

Reference: Daniel Harabor and Alban Grastien, “Improving Jump Point Search,”
International Conference on Automated Planning and Scheduling (ICAPS), 2014,
[paper](https://users.cecs.anu.edu.au/~dharabor/data/papers/harabor-grastien-icaps14.pdf).

Runnable example: [`examples/jps_grid.py`](../../examples/jps_grid.py).
Run it from the repository root with `python -m examples.jps_grid`.
