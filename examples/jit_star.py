"""JIT* with a planar manipulator and collision-guided sampling."""

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
        max_iters=1_000,
        step_size=0.7,
        goal_sample_rate=0.25,
        gamma=10.0,
        max_connection_radius=3.0,
        jit_ancestor_depth=8,
        jit_sample_radius=0.3,
        jit_bias_probability=0.5,
        manipulability_weight=0.001,
        allow_python_callbacks=True,
    ),
    seed=2,
)

print("success:", result.success)
print("objective cost:", result.stats.get("path_cost"))
print("collision-guided samples:", result.stats.get("jit_biased_samples"))
print("Jacobian evaluations:", result.stats.get("manipulability_evaluations"))
