# D* Lite

`plan_discrete(problem, planner="dstar_lite")` runs one native D* Lite query.
For repeated planning, construct `DStarLitePlanner(graph, start, goal)` and
reuse its `plan()`, `move_start()`, `update_edges()`, and `reset()` methods.
The native session owns adjacency, `g`/`rhs` values, and its priority queue.

The graph must have a stable directed-edge topology and non-negative costs.
Use positive infinity to block an existing edge and `None` to restore that
edge's initial cost. To update an undirected edge, pass both directed arcs.
A topology change requires constructing a new planner; `reset()` clears the
cached search state for the same topology. A graph heuristic must be
admissible and consistent; without one, D* Lite uses a zero heuristic. The
planner requires an exact goal node. `max_expansions` bounds work per call and
may return a valid incumbent with `MAX_ITERS` as its stop reason. Trace
recording is supported. `max_runtime_ms` limits native search time; a timeout
may return a valid incumbent with `TIME_BUDGET`.

The built-in 2D grid adapter materializes every in-bounds motion, including
blocked edges, so edge-cost updates can open or close them while preserving
their IDs. Custom graph adapters should include potentially changing edges in
their initial adjacency.

Reference: Sven Koenig and Maxim Likhachev, “D* Lite,” AAAI, 2002,
[paper](https://www.aaai.org/Papers/AAAI/2002/AAAI02-072.pdf).

Runnable example: [`examples/dstar_lite_grid.py`](../../examples/dstar_lite_grid.py).
Run it from the repository root with `python -m examples.dstar_lite_grid`.
