# Kinodynamic SIPP

`plan_temporal(problem, planner="kinodynamic_sipp")` runs Safe Interval Path
Planning with Interval Projection (SIPP-IP) on a configuration graph whose
states include scalar speed. Unlike standard SIPP, it propagates intervals of
reachable arrival times through nonzero-speed states, so it never inserts a
wait action where the robot cannot stop.

The temporal axis uses integer ticks. Node and edge blocked intervals still use
the half-open form `[start, end)`, with integer endpoints; `start_time` and an
optional `horizon` are integers, and the horizon is exclusive. Each edge is a
directed constant-acceleration motion primitive. Supply `node_velocities` for
every graph node, `edge_distances` for every directed edge, and positive finite
`max_speed`, `max_acceleration`, and `max_deceleration`. Primitive durations
come from `edge_durations` or graph edge costs and must be positive integer
ticks. The planner checks that distance equals average endpoint speed times
duration and that the implied acceleration fits the supplied limits. A node is
waitable exactly when its speed is zero.

The native search represents a state as a configuration and a reachable wait
interval contained in one safe node interval. It projects valid departure
intervals across each motion primitive, splitting them around edge conflicts
and destination safe intervals. The admissible static shortest-time heuristic
orders these states. With a finite graph and the stated discrete motion model,
the search is complete and returns an earliest-arrival plan when it finishes;
budget-limited incumbents are feasible but may not be optimal.

Reference: Zain Alabedeen Ali and Konstantin Yakovlev, “Safe Interval Path
Planning with Kinodynamic Constraints,” AAAI 2023,
[paper](https://ojs.aaai.org/index.php/AAAI/article/download/26453/26225).

Runnable example: [`examples/kinodynamic_sipp.py`](../../examples/kinodynamic_sipp.py).
