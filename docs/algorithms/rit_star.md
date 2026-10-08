# RIT*

RIT* is a batch-informed sampler for additive Riemannian arc length. It uses a
positive-definite metric tensor to price edges, define anisotropic connection
neighborhoods, and focus samples after it finds a path. A constant tensor uses
Cholesky whitening for informed sampling. A spatially varying tensor uses its
declared global lower eigenvalue bound to form a conservative Euclidean
informed set.

```python
import numpy as np

from pathplanning import RitParams, plan_continuous
from pathplanning.core.contracts import ContinuousProblem, GoalState
from pathplanning.spaces import AnisotropicContinuousSpace

space = AnisotropicContinuousSpace(
    lower_bound=[0.0, 0.0],
    upper_bound=[5.0, 5.0],
    metric=[[1.0, 0.0], [0.0, 4.0]],
    obstacles=(((2.0, 0.0), (3.0, 4.0)),),
)
problem = ContinuousProblem(space, [0.5, 2.5], GoalState([4.5, 2.5]))
result = plan_continuous(
    problem,
    planner="rit_star",
    params=RitParams(sample_count=512, batch_size=32, allow_python_callbacks=True),
    seed=7,
)
print(result.success, result.stats.get("metric_updates"))
```

Custom metric spaces implement `metric_tensor(state)` and provide global
`metric_eigenvalue_bounds = (lambda_min, lambda_max)` plus a
`metric_is_constant` boolean. The bounds must hold throughout the planning
domain. Custom metric, sampling, and collision callbacks require the explicit
`allow_python_callbacks=True` option. If no metric tensor is supplied, RIT*
uses the Euclidean identity metric.

RIT* estimates straight-edge Riemannian arc length with Gauss-Legendre
quadrature (order 1–10, default 10). It first checks the midpoint metric, then
a Simpson estimate, and evaluates the full quadrature and collision check only
for candidates that can still improve the incumbent. Spatially varying metric
estimates are deflated using the declared eigenvalue bounds before pruning.
`gamma` controls the anisotropic connection radius and `max_connection_radius`
caps it. `max_iters` counts candidate edge expansions; `sample_count` and
`batch_size` control the informed sample batches.

Collision-Adaptive Riemannian Metric (CARM) raises metric cost around observed
collisions with a Gaussian density field. The implementation records candidate
edge midpoints as collision feedback, applies updates every
`carm_update_interval` batches, and retains at most 4096 feedback points. Each
update recomputes tree costs before the next batch. `carm_sigma` sets the field
width and `carm_alpha` its maximum scale increase; set `carm_alpha=0` to disable
CARM.

Statistics include metric evaluations and updates, samples, edge expansions,
collision checks, rewires, and final Riemannian path cost. The implementation
uses a finite sample and iteration budget and does not claim asymptotic
optimality from finite tests. The reference anisotropic space and its custom
Python callbacks prioritize clarity over throughput.

Reference: [Din et al., RIT*: Riemannian Informed Trees for Cost-Adaptive
Optimal Motion Planning (2026, version 1)](https://arxiv.org/abs/2608.00822).
