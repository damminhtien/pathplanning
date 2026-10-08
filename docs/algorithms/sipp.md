# Safe Interval Path Planning

`plan_temporal(problem, planner="sipp")` searches a graph with known
time-varying node and edge blocks. `TemporalProblem` accepts directed edge
durations, a start time, optional time horizon, and blocked intervals. If an
edge duration is omitted, its graph edge cost is used.

Blocked intervals use the half-open convention `[start, end)`. The agent may
wait while its current node remains safe. Edge traversal occupies `[depart,
arrive)`, and the destination must be safe at arrival. `TemporalPlanResult`
contains `states` and absolute `times` for each arrival; `path_cost` is elapsed
time from `start_time`. Waiting is represented by the gap between one state's
arrival and the next state's arrival minus the edge duration.

Durations must be positive and finite. The goal is an exact graph node.
`max_expansions` and `max_runtime_ms` bound native search; a budget stop may
return a valid incumbent with `MAX_ITERS` or `TIME_BUDGET`. SIPP assumes
instantaneous stopping at nodes and permits waiting. Use the kinodynamic SIPP
variant when acceleration and velocity constraints matter.

Reference: Mike Phillips and Maxim Likhachev, “SIPP: Safe Interval Path
Planning for Dynamic Environments,” ICRA 2011,
[paper](https://www.cs.cmu.edu/~maxim/files/sipp_icra11.pdf).

Runnable example: [`examples/sipp_grid.py`](../../examples/sipp_grid.py).
Run it from the repository root with `python -m examples.sipp_grid`.
