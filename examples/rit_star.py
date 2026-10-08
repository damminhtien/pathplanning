"""RIT* with anisotropic cost and collision-adaptive refinement."""

import numpy as np

from pathplanning import RitParams, plan_continuous
from pathplanning.core.contracts import ContinuousProblem, GoalState
from pathplanning.spaces import AnisotropicContinuousSpace

space = AnisotropicContinuousSpace(
    lower_bound=[0.0, 0.0],
    upper_bound=[5.0, 5.0],
    metric=np.diag([1.0, 4.0]),
    obstacles=(((2.0, 0.0), (3.0, 4.0)),),
)
problem = ContinuousProblem(space, [0.5, 2.5], GoalState([4.5, 2.5]))
result = plan_continuous(
    problem,
    planner="rit_star",
    params=RitParams(
        sample_count=128,
        batch_size=16,
        max_iters=20_000,
        gamma=6.0,
        max_connection_radius=3.0,
        carm_update_interval=1,
        carm_sigma=0.3,
        carm_alpha=2.0,
        allow_python_callbacks=True,
    ),
    seed=13,
)

print("success:", result.success)
print("path cost:", result.stats.get("path_cost"))
print("metric updates:", result.stats.get("metric_updates"))
