"""Plan around an obstacle with the native AIT* planner."""

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

print("success:", result.success)
print("path cost:", result.stats.get("path_cost"))
