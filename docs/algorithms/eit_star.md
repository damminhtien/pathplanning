# EIT*

EIT* (Effort Informed Trees) extends the asymmetric reverse/forward search
pattern with an estimate of collision-checking effort. This implementation
computes a reverse shortest-path estimate in expected collision samples, then
uses that estimate as a secondary priority after the admissible Euclidean path
cost. Edges already validated in the current call have zero remaining
validation effort; unknown edges estimate checks from `collision_step`.

The native planner shares the fixed random geometric graph and full reverse
recomputation behavior of this repository's AIT* implementation. It does not
implement EIT*'s adaptive sparse collision validation, explicit-estimation
queues, or repeated sample batches, so it does not claim the paper's
asymptotic-optimality guarantee. It supports exact point goals, symmetric
motion validity, Euclidean distance, and additive path length. Custom
objectives and non-Euclidean metrics are rejected.

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
    planner="eit_star",
    params=RrtParams(
        sample_count=128,
        step_size=0.5,
        collision_step=0.1,
        rrt_star_radius_gamma=8.0,
        rrt_star_radius_max_factor=12.0,
    ),
    seed=13,
)
print(result.success, result.stats.get("motion_checks"))
```

Reference: [Strub and Gammell, AIT* and EIT*: Asymmetric bidirectional
sampling-based path planning (IJRR 2022)](https://arxiv.org/abs/2111.01877).

Runnable example: [`examples/eit_star.py`](../../examples/eit_star.py).
Run from the repository root with `python -m examples.eit_star`.
