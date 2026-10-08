"""Plan through a complete informed graph with batched edge validation."""

from pathplanning import RrtParams, plan_continuous
from pathplanning.core.contracts import ContinuousProblem, GoalState
from pathplanning.spaces.continuous_3d import ContinuousSpace3D

space = ContinuousSpace3D(lower_bound=[0, 0, 0], upper_bound=[10, 10, 10])
problem = ContinuousProblem(
    space=space,
    start=[1.0, 1.0, 1.0],
    goal=GoalState([9.0, 9.0, 1.0]),
)
result = plan_continuous(
    problem,
    planner="fcit_star",
    params=RrtParams(sample_count=256, batch_size=32),
    seed=7,
)
print("success:", result.success)
print("motion checks:", result.stats.get("motion_checks"))
