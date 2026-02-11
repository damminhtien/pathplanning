"""Typed parameter objects for reusable sampling-based planners."""

from __future__ import annotations

from dataclasses import dataclass
import math


def _is_valid_real(value: object) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


@dataclass(slots=True)
class RrtParams:
    """Runtime parameters shared by RRT-family planners."""

    max_iters: int = 10_000
    step_size: float = 0.5
    goal_sample_rate: float = 0.05
    time_budget_s: float | None = None
    max_sample_tries: int = 1_000
    collision_step: float = 0.1
    goal_reach_tolerance: float = 1e-9
    rrt_star_radius_gamma: float = 2.0
    rrt_star_radius_bias: float = 1.0
    rrt_star_radius_max_factor: float = 6.0

    def __post_init__(self) -> None:
        self.validate()

    def validate(self) -> RrtParams:
        """Validate parameter values and return ``self`` for chaining."""
        if type(self.max_iters) is not int:
            raise TypeError("max_iters must be an integer")
        if self.max_iters <= 0:
            raise ValueError("max_iters must be > 0")

        if not _is_valid_real(self.step_size):
            raise TypeError("step_size must be a finite real number")
        if self.step_size <= 0:
            raise ValueError("step_size must be > 0")

        if not _is_valid_real(self.goal_sample_rate):
            raise TypeError("goal_sample_rate must be a finite real number")
        if not 0.0 <= float(self.goal_sample_rate) <= 1.0:
            raise ValueError("goal_sample_rate must be in [0, 1]")

        if self.time_budget_s is not None:
            if not _is_valid_real(self.time_budget_s):
                raise TypeError("time_budget_s must be a finite real number or None")
            if float(self.time_budget_s) <= 0.0:
                raise ValueError("time_budget_s must be > 0 when provided")

        if type(self.max_sample_tries) is not int:
            raise TypeError("max_sample_tries must be an integer")
        if self.max_sample_tries <= 0:
            raise ValueError("max_sample_tries must be > 0")

        if not _is_valid_real(self.collision_step):
            raise TypeError("collision_step must be a finite real number")
        if self.collision_step <= 0:
            raise ValueError("collision_step must be > 0")

        if not _is_valid_real(self.goal_reach_tolerance):
            raise TypeError("goal_reach_tolerance must be a finite real number")
        if self.goal_reach_tolerance < 0:
            raise ValueError("goal_reach_tolerance must be >= 0")

        if not _is_valid_real(self.rrt_star_radius_gamma):
            raise TypeError("rrt_star_radius_gamma must be a finite real number")
        if self.rrt_star_radius_gamma <= 0:
            raise ValueError("rrt_star_radius_gamma must be > 0")

        if not _is_valid_real(self.rrt_star_radius_bias):
            raise TypeError("rrt_star_radius_bias must be a finite real number")
        if self.rrt_star_radius_bias < 0:
            raise ValueError("rrt_star_radius_bias must be >= 0")

        if not _is_valid_real(self.rrt_star_radius_max_factor):
            raise TypeError("rrt_star_radius_max_factor must be a finite real number")
        if self.rrt_star_radius_max_factor <= 0:
            raise ValueError("rrt_star_radius_max_factor must be > 0")

        return self
