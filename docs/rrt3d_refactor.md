# Dynamic RRT3D

This note documents the current stateful 3D RRT API and its native backend.
The Python-facing planner lives in
`pathplanning/planners/sampling/dynamic_rrt.py`; the C implementation is
`pp_dynamic_rrt_plan` in `pathplanning/native/continuous_engine.c`.

## Ownership and Execution

`DynamicRRT3D` keeps the existing Python API and public tree views (`nodes`,
`parent_by_node`, `edges`, and `node_state`). `grow_rrt`, `trim_rrt`,
`regrow_rrt`, and `find_affected_edges` serialize the tree as state and parent
arrays, call the native C planner, then rebuild the Python views from returned
arrays.

The C engine performs pruning, incoming-edge validity checks, nearest-node
queries, sampling, steering, tree growth, and goal connection. It uses
contiguous state and parent arrays with an incremental KD forest. The optional
`NearestNodeIndex` implementation still serves direct Python calls to
`planner.nearest(...)`; the native growth loop does not call that Python index.

The built-in `ContinuousSpace3D` representation is copied into native bounds
and obstacle arrays before planning. A custom `ContinuousSpace` uses the
compatibility callbacks for sampling, state checks, motion checks, distance,
and steering, so those operations can enter Python during a native run.

`find_affected_edges(obstacle)` inspects the planner's current `space`; it does
not apply the `obstacle` argument itself. Update the environment's obstacle
model first, then call `find_affected_edges`, `invalidate_nodes`, `trim_rrt`, or
`regrow_rrt` as needed.

## Basic Usage

Build the native extensions before running from a source checkout:

```bash
make build-ext
```

Create a reproducible planner with an explicit space and RNG seed:

```python
from pathplanning.planners.sampling.dynamic_rrt import DynamicRRT3D, DynamicRRT3DConfig
from pathplanning.spaces.continuous_3d import ContinuousSpace3D

space = ContinuousSpace3D(lower_bound=[0, 0, 0], upper_bound=[20, 20, 6])
config = DynamicRRT3DConfig(step_size=0.25, max_iterations=10_000)
planner = DynamicRRT3D.with_seed(7, environment=space, config=config)
planner.init_rrt()
planner.grow_rrt()
path_result = planner.path()
```

`path_result` is `None` when the goal was not connected. When connected, it
contains the goal-to-start edge list and its accumulated Euclidean length.

## Configuration

`DynamicRRT3DConfig` contains step size, iteration limit, goal and waypoint
sampling probabilities, a legacy dynamic-step setting, and the rebuild
threshold for the Python-facing nearest-neighbor helper. The native planner
uses step size, iteration limit, and the sampling probabilities; its KD forest
has its own native batching strategy.

`DynamicRRT3D` is a stateful convenience API and is not a registry entry. The
registry-based `rrt` planner uses the standard `ContinuousProblem` and
`plan_continuous(...)` API instead. See
[`native_core.md`](native_core.md) for the native ABI, supported spaces, and
callback boundary.

## Visualization

`DynamicRRT3D.grow_rrt`, `regrow_rrt`, and `trim_rrt` accept an optional
`trace=TraceOptions()` and return the trace for that native run. The planner
also stores it as `last_trace`. Rendering uses `pathplanning.viz` after the run;
see [the visualization guide](visualization.md).
