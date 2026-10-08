"""Run Ackermann and differential-drive state-lattice searches."""

from __future__ import annotations

import numpy as np

from pathplanning import StateLatticeParams, plan_continuous
from pathplanning.core.contracts import ContinuousProblem, GoalState
from pathplanning.spaces import (
    StateLatticeGridSpace,
    ackermann_motion_primitives,
    differential_drive_motion_primitives,
)


def main() -> None:
    occupancy = np.zeros((20, 20), dtype=bool)
    ackermann = StateLatticeGridSpace(
        occupancy,
        resolution=1.0,
        primitives=ackermann_motion_primitives(
            wheelbase=1.0,
            max_steering_angle=0.5,
            primitive_length=1.0,
            collision_step=0.2,
        ),
        footprint_length=0.8,
        footprint_width=0.5,
        rotation_radius=1.0 / np.tan(0.5),
    )
    differential = StateLatticeGridSpace(
        occupancy,
        resolution=1.0,
        primitives=differential_drive_motion_primitives(
            track_width=0.6,
            primitive_length=1.0,
            collision_step=0.2,
        ),
        footprint_length=0.8,
        footprint_width=0.5,
        rotation_radius=0.3,
    )

    for model_name, space in (("Ackermann", ackermann), ("Differential drive", differential)):
        problem = ContinuousProblem(space, (2.5, 2.5, 0.0), GoalState((8.5, 2.5, 0.0)))
        result = plan_continuous(
            problem,
            planner="state_lattice",
            params=StateLatticeParams(goal_xy_tolerance=0.1, goal_yaw_tolerance=0.1),
        )
        if not result.success or result.path is None:
            raise RuntimeError(f"{model_name} state-lattice search failed: {result.stop_reason}")
        print(f"{model_name}: {len(result.path)} poses, cost {result.stats['path_cost']:.2f}")


if __name__ == "__main__":
    main()
