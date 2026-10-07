# Lazy Theta*

`plan_discrete(problem, planner="lazy_theta_star")` runs native Lazy Theta* on
an eight-connected `Grid2DSearchSpace` or `TerrainCostGrid2D`. It returns cell
waypoints joined by any-angle segments. Candidate parent shortcuts are assumed
visible during relaxation, then checked when their destination is removed from
the queue. If a shortcut is blocked, the planner reparents through the
lowest-cost closed adjacent cell with a valid move.

Visibility checks reject blocked cells and diagonal corner cutting. Uniform
segments cost their Euclidean length; terrain segments integrate the positive
per-cell cost along the segment. The heuristic uses the global minimum terrain
cost. The planner requires an exact cell goal and standard eight-connected
motions. `max_expansions` must be a positive integer. Its result is a feasible
any-angle route, not a guarantee of the globally shortest continuous route.
The algorithm is deterministic; its `rng` argument is unused. Native trace
recording is supported.

Reference: Alex Nash, Sven Koenig, and Craig Tovey, “Lazy Theta*: Any-Angle
Path Planning and Path Length Analysis in 3D,” AAAI, 2010,
[paper](https://ojs.aaai.org/index.php/AAAI/article/view/7566).

Runnable example: [`examples/lazy_theta_star_grid.py`](../../examples/lazy_theta_star_grid.py).
Run it from the repository root with `python -m examples.lazy_theta_star_grid`.
