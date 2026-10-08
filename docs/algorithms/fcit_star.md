# FCIT*

FCIT* (Fully Connected Informed Trees) searches a complete directed graph over
each sample batch. It keeps one locally ordered outgoing-edge queue per reached
vertex and places only that queue's next useful edge in the shared priority
queue. Edges are ordered by the admissible estimate `g(source) + d(source,
target) + d(target, goal)`. Invalid edges are cached by direction, and the
search restarts on each new informed sample batch while retaining the valid tree.

```python
from pathplanning import RrtParams, plan_continuous
from pathplanning.core.contracts import ContinuousProblem, GoalState
from pathplanning.spaces.continuous_3d import ContinuousSpace3D

space = ContinuousSpace3D(lower_bound=[0, 0, 0], upper_bound=[10, 10, 10])
problem = ContinuousProblem(
    space,
    [1.0, 1.0, 1.0],
    GoalState([9.0, 9.0, 1.0]),
)
result = plan_continuous(
    problem,
    planner="fcit_star",
    params=RrtParams(sample_count=256, batch_size=32),
    seed=7,
)
print(result.success, result.stats.get("motion_checks"))
```

The planner requires an exact point goal, Euclidean distance, and additive path
length. It evaluates candidate motions in batches of up to 64 edges, with the
configured `batch_size` as the upper bound. Custom spaces can provide
`is_motion_valid_batch_with_step(edges, collision_step)` to process a batch in a
single callback; the scalar motion checker remains the portable fallback.

This implementation has a finite `sample_count` budget and makes no
asymptotic-optimality claim. Native built-in space models currently use the
portable scalar collision checker inside each candidate batch. The FCIT* paper
describes hardware-accelerated batch evaluation and asymptotic convergence as
the sample set grows.

Reference: [Wilson et al., Nearest-Neighbourless Asymptotically Optimal Motion
Planning with Fully Connected Informed Trees (ICRA 2025)](https://kavrakilab.org/publications/wilson2025-fcit.pdf).
