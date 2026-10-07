# Theta*

`plan_discrete(problem, planner="theta_star")` runs the native C++ Theta*
search on a standard eight-connected grid and returns the cell waypoints of
its any-angle path. The line segments between consecutive waypoints are checked
against the occupancy grid. A segment that touches a blocked cell or passes
through a grid corner with a blocked side cell is rejected.

`Grid2DSearchSpace` uses unit terrain cost, so a visible segment costs its
Euclidean length. `TerrainCostGrid2D` integrates its positive per-cell terrain
cost along the portion of the segment inside each cell. Theta* uses a global
minimum-terrain Euclidean heuristic. It requires exact cell goals and the
standard eight-connected motions. `max_expansions` limits expanded grid cells
and must be a positive integer. The algorithm is deterministic; its `rng`
argument is unused. Native trace recording is supported.

Theta* is an any-angle heuristic search, not an exact shortest-path solver.
Its line-of-sight and waypoint contract is checked against an independent
cell-intersection oracle in the tests.

For weighted maps, the reported cost is the line integral of terrain across
the returned segments. The algorithm does not guarantee a globally shortest
route under that objective.

Reference: Alex Nash, Kenny Daniel, Sven Koenig, and Ariel Felner, “Theta*:
Any-Angle Path Planning on Grids,” AAAI, 2007,
[paper](https://aaai.org/Papers/AAAI/2007/aaai07-187.pdf).

Runnable example: [`examples/theta_star_grid.py`](../../examples/theta_star_grid.py).
Run it from the repository root with `python -m examples.theta_star_grid`.
