# Bounded-Suboptimal SIPP

`plan_temporal(problem, planner="bounded_suboptimal_sipp")` uses FocalSIPP on
the same temporal graph contract as SIPP. Set `params={"w": 1.5}` to bound a
completed solution's elapsed travel time by `w` times the optimal time. The
default is `w=1.5`; values must be finite and at least 1.

The native search maintains OPEN ordered by an admissible reverse shortest-path
time estimate. FOCAL contains OPEN states with `f <= w * f_min`; it selects the
state with the fewest remaining graph edges. The search supports state
re-expansion when a lower arrival time is found. When an expansion or time
budget stops the search, any returned incumbent is feasible, but the `w` bound
is guaranteed only when `stop_reason` is `success`.

This planner retains SIPP's graph, half-open blocked-interval, waiting, and
exact-goal requirements. It assumes instantaneous stopping at nodes.

Reference: Konstantin Yakovlev, Anton Andreychuk, and Roni Stern,
“Revisiting Bounded-Suboptimal Safe Interval Path Planning,” ICAPS 2020,
[paper](https://ojs.aaai.org/index.php/ICAPS/article/download/6674/6528/9903).

Runnable example: [`examples/bounded_suboptimal_sipp.py`](../../examples/bounded_suboptimal_sipp.py).
