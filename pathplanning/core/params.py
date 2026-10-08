"""Typed parameter objects for reusable sampling-based planners."""

from __future__ import annotations

from collections.abc import Iterator, Mapping
from dataclasses import dataclass
import math


def _is_valid_real(value: object) -> bool:
    if isinstance(value, bool):
        return False
    if not isinstance(value, (int, float)):
        return False
    return math.isfinite(float(value))


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
    sample_count: int = 512
    batch_size: int = 64
    abit_inflation_parameter: float = 10.0
    abit_truncation_parameter: float = 5.0
    allow_python_callbacks: bool = False

    def __post_init__(self) -> None:
        self.validate()

    def validate(self) -> RrtParams:
        """Validate parameter values and return ``self`` for chaining."""
        if type(self.allow_python_callbacks) is not bool:
            raise TypeError("allow_python_callbacks must be a bool")

        if isinstance(self.max_iters, bool) or type(self.max_iters) is not int:
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

        if isinstance(self.max_sample_tries, bool) or type(self.max_sample_tries) is not int:
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

        for name in ("sample_count", "batch_size"):
            value = getattr(self, name)
            if isinstance(value, bool) or type(value) is not int:
                raise TypeError(f"{name} must be an integer")
            if value <= 0:
                raise ValueError(f"{name} must be > 0")

        if not _is_valid_real(self.abit_inflation_parameter):
            raise TypeError("abit_inflation_parameter must be a finite real number")
        if self.abit_inflation_parameter < 0:
            raise ValueError("abit_inflation_parameter must be >= 0")

        if not _is_valid_real(self.abit_truncation_parameter):
            raise TypeError("abit_truncation_parameter must be a finite real number")
        if self.abit_truncation_parameter < 0:
            raise ValueError("abit_truncation_parameter must be >= 0")

        return self


@dataclass(slots=True)
class RoadmapParams(Mapping[str, object]):
    """Sampling, connection, and query limits for reusable PRM* roadmaps."""

    sample_count: int = 512
    gamma: float = 2.0
    max_sample_tries: int = 1_000
    max_expansions: int = 100_000
    collision_step: float = 0.1
    time_budget_s: float | None = None
    allow_python_callbacks: bool = False

    def __getitem__(self, key: str) -> object:
        if key not in self.__dataclass_fields__:
            raise KeyError(key)
        return getattr(self, key)

    def __iter__(self) -> Iterator[str]:
        return iter(self.__dataclass_fields__)

    def __len__(self) -> int:
        return len(self.__dataclass_fields__)

    def validate(self) -> RoadmapParams:
        """Validate values before a native roadmap call."""
        if type(self.allow_python_callbacks) is not bool:
            raise TypeError("allow_python_callbacks must be a bool")
        for name in ("sample_count", "max_sample_tries", "max_expansions"):
            value = getattr(self, name)
            if isinstance(value, bool) or type(value) is not int:
                raise TypeError(f"{name} must be an integer")
            if value <= 0:
                raise ValueError(f"{name} must be > 0")
        if not _is_valid_real(self.gamma):
            raise TypeError("gamma must be a finite real number")
        if self.gamma <= 0:
            raise ValueError("gamma must be > 0")
        if not _is_valid_real(self.collision_step):
            raise TypeError("collision_step must be a finite real number")
        if self.collision_step <= 0:
            raise ValueError("collision_step must be > 0")
        if self.time_budget_s is not None:
            if not _is_valid_real(self.time_budget_s):
                raise TypeError("time_budget_s must be a finite real number or None")
            if self.time_budget_s <= 0:
                raise ValueError("time_budget_s must be > 0 when provided")
        return self


@dataclass(slots=True)
class RitParams(Mapping[str, object]):
    """Sampling, metric-refinement, and collision limits for RIT*."""

    max_iters: int = 10_000
    sample_count: int = 512
    batch_size: int = 64
    max_sample_tries: int = 1_000
    step_size: float = 0.5
    collision_step: float = 0.1
    time_budget_s: float | None = None
    gamma: float = 2.0
    max_connection_radius: float = 3.0
    quadrature_order: int = 10
    carm_update_interval: int = 15
    carm_sigma: float = 0.1
    carm_alpha: float = 10.0
    allow_python_callbacks: bool = False

    def __getitem__(self, key: str) -> object:
        if key not in self.__dataclass_fields__:
            raise KeyError(key)
        return getattr(self, key)

    def __iter__(self) -> Iterator[str]:
        return iter(self.__dataclass_fields__)

    def __len__(self) -> int:
        return len(self.__dataclass_fields__)

    def validate(self) -> RitParams:
        """Validate RIT* limits and CARM parameters."""
        if type(self.allow_python_callbacks) is not bool:
            raise TypeError("allow_python_callbacks must be a bool")
        for name in (
            "max_iters",
            "sample_count",
            "batch_size",
            "max_sample_tries",
            "carm_update_interval",
        ):
            value = getattr(self, name)
            if isinstance(value, bool) or type(value) is not int:
                raise TypeError(f"{name} must be an integer")
            if value <= 0:
                raise ValueError(f"{name} must be > 0")
        if isinstance(self.quadrature_order, bool) or type(self.quadrature_order) is not int:
            raise TypeError("quadrature_order must be an integer")
        if not 1 <= self.quadrature_order <= 10:
            raise ValueError("quadrature_order must be between 1 and 10")
        for name in (
            "step_size",
            "collision_step",
            "gamma",
            "max_connection_radius",
            "carm_sigma",
        ):
            value = getattr(self, name)
            if not _is_valid_real(value):
                raise TypeError(f"{name} must be a finite real number")
            if value <= 0:
                raise ValueError(f"{name} must be > 0")
        if not _is_valid_real(self.carm_alpha):
            raise TypeError("carm_alpha must be a finite real number")
        if self.carm_alpha < 0:
            raise ValueError("carm_alpha must be >= 0")
        if self.time_budget_s is not None:
            if not _is_valid_real(self.time_budget_s):
                raise TypeError("time_budget_s must be a finite real number or None")
            if self.time_budget_s <= 0:
                raise ValueError("time_budget_s must be > 0 when provided")
        return self
