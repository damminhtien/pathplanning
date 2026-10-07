# Weighted Jump Point Search

`plan_discrete(problem, planner="jpsw")` runs the native C++ Weighted Jump
Point Search kernel on a terrain-cost grid. The search operates directly on
the grid snapshots and returns every traversed cell in its path.

Build the graph with `TerrainCostGrid2D`. Terrain values must be finite and
strictly positive. A cardinal move costs the average terrain value of its two
endpoint cells. A diagonal move costs √2 times the average value of the four
cells touched by the move. Diagonal corner cutting is prohibited. The planner
requires the standard eight-connected motions and an exact cell goal; it
rejects custom edge-cost overrides. `max_expansions` limits expanded jump
points and must be a positive integer. The search is deterministic, so its
`rng` argument is unused. Native trace recording is supported.

The implementation follows the paper's weighted neighborhood pruning and
orthogonal-last tiebreak, stops jumps at terrain boundaries, and uses diagonal
branch plus prospective-g pruning to reduce redundant scans. It does not use
the paper's optional jump cache, so repeated diagonal scans can still be costly
on maps with broad terrain transitions. The paper proves optimality for its
stated weighted-grid model. This implementation additionally enforces strict
no-corner-cutting movement; tests compare its result against an independent
Dijkstra oracle on fixed and seeded-random grids, which is empirical coverage
of that adaptation rather than a separate proof.

The terrain integration model averages all cells intersected by a move. Maps
whose terrain weights are destination-only or use another edge model are not
supported by this planner. Grid path optimality is relative to this discrete
edge-cost model, not continuous weighted-region path length.

Reference: Mark Carlson, Sajjad K. Moghadam, Daniel D. Harabor, Peter J.
Stuckey, and Morteza Ebrahimi, “Optimal Pathfinding on Weighted Grid Maps,”
AAAI-23, 2023, [paper](https://pathfinding.ai/pdf/cmhse-aaai23-jpsw.pdf).

Runnable example: [`examples/jpsw_grid.py`](../../examples/jpsw_grid.py).
Run it from the repository root with `python -m examples.jpsw_grid`.
