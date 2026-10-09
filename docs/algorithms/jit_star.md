# JIT*

JIT* combines just-in-time edge expansion, collision-guided sampling, and a
manipulability-aware path objective. The native search examines a bounded set
of ancestors beyond the local connection neighborhood. When an edge fails
collision checking, later samples are drawn in a corridor around that edge and
filtered against the current informed set. A nonzero uniform-sampling
probability remains so the planner continues exploring the full free space.

```python
import numpy as np

from pathplanning import JitParams, plan_continuous
from pathplanning.core.contracts import ContinuousProblem, GoalState
from pathplanning.spaces import PlanarManipulatorSpace

space = PlanarManipulatorSpace(
    joint_lower=[-2.4, -2.4, -2.4],
    joint_upper=[2.4, 2.4, 2.4],
    link_lengths=[1.0, 0.8, 0.6],
    obstacles=((2.0, 0.0, 0.15),),
)
problem = ContinuousProblem(
    space,
    np.array([-0.8, 0.5, 0.7]),
    GoalState(np.array([0.8, -0.5, -0.7])),
)
result = plan_continuous(
    problem,
    planner="jit_star",
    params=JitParams(
        sample_count=96,
        batch_size=32,
        manipulability_weight=0.001,
        allow_python_callbacks=True,
    ),
    seed=2,
)
print(result.success, result.stats.get("manipulability_evaluations"))
```

With the default positive `manipulability_weight`, the space must implement
`jacobian(state)` and return a finite matrix with one column per joint. Python
computes its smallest singular value with SVD; the native engine applies the
paper's
`D_tanh = tanh(eta / (sigma_min + epsilon)) / (sigma_min + epsilon)` penalty
using Simpson quadrature over each candidate edge. The weight scales the
nonnegative penalty added to Euclidean edge length. Setting the weight to zero
disables manipulability scoring and permits ordinary Euclidean spaces.

Custom state-validity, motion, and Jacobian callbacks require
`allow_python_callbacks=True`. Robot spaces should include self-collision and
joint-limit rules in `is_state_valid` and validate the full motion in
`is_motion_valid`; the included planar serial-arm model checks non-adjacent
links and circular obstacles. The goal is an exact configuration and remains
fixed; this API does not perturb a goal in the Jacobian null space.

`jit_ancestor_depth` bounds ancestor candidates, `jit_sample_count` bounds
attempts around a failed edge, `jit_sample_radius` controls corridor width, and
`jit_bias_probability` controls how often corridor samples replace uniform or
informed samples. `sample_count` and `batch_size` define the sample budget and
reported batches. Statistics include biased samples, ancestor candidates,
Jacobian evaluations, collision checks, and final objective cost. The native
implementation uses a forward RRT* tree with these JIT refinements and does not
claim the paper's asymptotic guarantee for finite budgets.

Reference: [Cai et al., Just in time Informed Trees: Manipulability-Aware
Asymptotically Optimized Motion Planning (2026, arXiv v1)](
https://arxiv.org/abs/2601.19972).

Runnable example: [`examples/jit_star.py`](../../examples/jit_star.py).
Run from the repository root with `python -m examples.jit_star`.
