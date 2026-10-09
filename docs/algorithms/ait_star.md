# AIT*

AIT* (Adaptively Informed Trees) uses an asymmetric search: a reverse search
builds an admissible cost-to-go estimate over sampled states, and a forward
edge-queue search uses that estimate to prioritize collision checks. When a
forward edge is blocked, the planner excludes it from the reverse graph and
repairs the estimate before continuing.

The current native implementation takes one finite random geometric graph per
call. It recomputes reverse shortest paths after a blocked edge rather than
using the reference LPA* incremental queue, and it does not add more sample
batches during that call. Results are valid paths for this sampled graph; the
implementation does not claim AIT*'s almost-sure asymptotic-optimality
guarantee. It supports exact point goals, symmetric motion validity, Euclidean
distance, and additive path length. Custom objectives and non-Euclidean metrics
are rejected.

```python
from pathplanning.api import plan_continuous
from pathplanning.core.contracts import ContinuousProblem, GoalState
from pathplanning.core.params import RrtParams
from pathplanning.spaces.grid2d import Grid2DSamplingSpace

space = Grid2DSamplingSpace(
    x_range=(0.0, 5.0),
    y_range=(0.0, 5.0),
    obs_rectangle=([2, 0, 1, 4],),
    delta=0.0,
)
problem = ContinuousProblem(space, (0.5, 2.5), GoalState((4.5, 2.5)))
result = plan_continuous(
    problem,
    planner="ait_star",
    params=RrtParams(
        sample_count=128,
        step_size=0.5,
        rrt_star_radius_gamma=8.0,
        rrt_star_radius_max_factor=12.0,
    ),
    seed=13,
)
print(result.success, result.stats.get("path_cost"))
```

Reference: [Strub and Gammell, AIT* and EIT*: Asymmetric bidirectional
sampling-based path planning (IJRR 2022)](https://arxiv.org/abs/2111.01877).

Runnable example: [`examples/ait_star.py`](../../examples/ait_star.py).
Run from the repository root with `python -m examples.ait_star`.
