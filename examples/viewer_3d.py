"""Replay a 3D RRT-Connect search after planning has finished."""

from __future__ import annotations

import numpy as np

from pathplanning import TraceOptions
from pathplanning.api import plan_continuous
from pathplanning.core.contracts import ContinuousProblem, GoalState
from pathplanning.spaces import AABB, ContinuousSpace3D, Sphere
from pathplanning.viz import scene_from_problem, view_result


def main() -> None:
    space = ContinuousSpace3D(
        lower_bound=np.array([0.0, 0.0, 0.0]),
        upper_bound=np.array([12.0, 12.0, 5.0]),
        aabbs=(AABB(np.array([5.0, 3.0, 0.0]), np.array([7.0, 9.0, 3.2])),),
        spheres=(Sphere(np.array([3.0, 8.0, 2.0]), 1.0),),
    )
    start = np.array([1.0, 1.0, 1.0])
    goal = np.array([11.0, 11.0, 1.0])
    problem = ContinuousProblem(
        space=space,
        start=start,
        goal=GoalState(state=goal),
    )
    result = plan_continuous(
        problem,
        planner="rrt_connect",
        params={"max_iters": 1500, "step_size": 0.7, "goal_sample_rate": 0.15},
        trace=TraceOptions(),
        seed=7,
    )
    viewer = view_result(scene_from_problem(problem), result)
    viewer.show(block=True)


if __name__ == "__main__":
    main()
