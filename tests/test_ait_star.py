"""Focused checks for native AIT* collision repair."""

from __future__ import annotations

from pathplanning.api import plan_continuous
from pathplanning.core.contracts import ContinuousProblem, GoalState
from pathplanning.core.params import RrtParams
from pathplanning.spaces.grid2d import Grid2DSamplingSpace


def test_ait_star_repairs_heuristic_after_blocked_edge() -> None:
    space = Grid2DSamplingSpace(
        x_range=(0.0, 5.0),
        y_range=(0.0, 5.0),
        obs_rectangle=([2, 0, 1, 4],),
        delta=0.0,
    )
    result = plan_continuous(
        ContinuousProblem(space, (0.5, 2.5), GoalState((4.5, 2.5))),
        planner="ait_star",
        params=RrtParams(
            sample_count=128,
            max_iters=20_000,
            step_size=0.5,
            collision_step=0.1,
            rrt_star_radius_gamma=8.0,
            rrt_star_radius_max_factor=12.0,
        ),
        seed=13,
    )

    assert result.success
    assert result.path is not None
    assert result.stats["motion_checks"] > 0
    assert all(space.is_motion_valid(a, b) for a, b in zip(result.path, result.path[1:]))
