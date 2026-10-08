# Hybrid A*

`plan_continuous(problem, planner="hybrid_astar")` plans a curvature-bounded
SE(2) path for an Ackermann-like vehicle. The search stores continuous poses
and groups them by discretized `(x, y, heading, gear)` keys. Its native C++
kernel expands straight, left, and right bicycle-model primitives in forward
and, by default, reverse. Each motion is sampled at `collision_step`; the
built-in grid model checks the complete rectangular vehicle footprint against
occupied cells and map boundaries.

The built-in `AckermannGridSpace` takes a two-dimensional Boolean occupancy
grid where `True` means occupied, plus its world resolution and origin. Vehicle
geometry is set with positive `wheelbase`, `footprint_length`, and
`footprint_width`, and `max_steering_angle` must be between zero and 90 degrees.
States and exact goals are `(x, y, yaw)` in world units and radians. The pose
reference is the center of the rectangular footprint.

```python
import numpy as np

from pathplanning import HybridAStarParams, plan_continuous
from pathplanning.core.contracts import ContinuousProblem, GoalState
from pathplanning.spaces import AckermannGridSpace

occupancy = np.zeros((20, 20), dtype=bool)
occupancy[10, 10] = True
space = AckermannGridSpace(
    occupancy,
    resolution=1.0,
    footprint_length=0.7,
    footprint_width=0.4,
)
problem = ContinuousProblem(space, (3.5, 10.5, 0.0), GoalState((16.5, 10.5, 0.0)))
result = plan_continuous(
    problem,
    planner="hybrid_astar",
    params=HybridAStarParams(max_expansions=30_000, primitive_length=1.0),
)
if not result.success:
    raise RuntimeError(f"planning stopped: {result.stop_reason.value}")
print(result.path)
print(result.directions)  # first pose is 0, then +1 forward or -1 reverse
```

When reverse is allowed, the planner tries a collision-checked Reeds-Shepp
connector; with reverse disabled it tries a Dubins connector. A connector is
accepted only after checking the footprint along every sampled pose. The
returned path contains sampled poses and a direction value per pose, so callers
can preserve forward/reverse motion. The search may also stop within
`goal_xy_tolerance` and `goal_yaw_tolerance`; analytic connectors terminate at
the exact requested goal.

`HybridAStarParams` controls the expansion and time budgets, XY and heading
discretization, primitive length, collision sampling, goal tolerances, analytic
expansion range/frequency, heuristic weight, and reverse/steering/switch
penalties. `heuristic_weight` and `reverse_penalty` must be at least 1. The
heuristic weight above 1 trades search effort for solution quality. This
discretized, first-solution implementation does not promise a continuous-space
optimality bound or completeness under finite resolution and budgets.

Custom kinematic spaces implement `wheelbase`, `max_steering_angle`, vehicle
footprint dimensions, and `is_state_valid(state)`. Their Python validity
callback is disabled by default; set `allow_python_callbacks=True` explicitly
to use it. Custom objectives are rejected because the planner's cost is its
native travel, reverse, steering, and direction-switch cost. Built-in grid
planning stays in native code.

References: [Dolgov et al., “Practical Search Techniques in Path Planning for
Autonomous Driving,” STAIR 2008](https://ai.stanford.edu/~ddolgov/papers/dolgov_gpp_stair08.pdf).
The Reeds-Shepp connector formulas are adapted from OMPL's
[ReedsSheppStateSpace.cpp](https://ompl.kavrakilab.org/ReedsSheppStateSpace_8cpp_source.html);
the adapted source retains its BSD 3-Clause notice in
[`pathplanning/THIRD_PARTY_NOTICES.md`](../../pathplanning/THIRD_PARTY_NOTICES.md).
This package has no OMPL runtime dependency.
