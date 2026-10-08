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


@dataclass(slots=True)
class JitParams(Mapping[str, object]):
    """Sampling, connectivity, and motion-performance parameters for JIT*."""

    max_iters: int = 10_000
    sample_count: int = 512
    batch_size: int = 64
    max_sample_tries: int = 1_000
    step_size: float = 0.5
    goal_sample_rate: float = 0.05
    collision_step: float = 0.1
    goal_reach_tolerance: float = 1e-9
    time_budget_s: float | None = None
    gamma: float = 2.0
    max_connection_radius: float = 3.0
    jit_ancestor_depth: int = 8
    jit_sample_count: int = 4
    jit_sample_radius: float = 0.25
    jit_bias_probability: float = 0.35
    manipulability_weight: float = 1.0
    manipulability_eta: float = 0.1
    manipulability_epsilon: float = 1e-6
    allow_python_callbacks: bool = False

    def __getitem__(self, key: str) -> object:
        if key not in self.__dataclass_fields__:
            raise KeyError(key)
        return getattr(self, key)

    def __iter__(self) -> Iterator[str]:
        return iter(self.__dataclass_fields__)

    def __len__(self) -> int:
        return len(self.__dataclass_fields__)

    def validate(self) -> JitParams:
        """Validate budgets and motion-performance weights."""
        if type(self.allow_python_callbacks) is not bool:
            raise TypeError("allow_python_callbacks must be a bool")
        for name in (
            "max_iters",
            "sample_count",
            "batch_size",
            "max_sample_tries",
            "jit_ancestor_depth",
            "jit_sample_count",
        ):
            value = getattr(self, name)
            if isinstance(value, bool) or type(value) is not int:
                raise TypeError(f"{name} must be an integer")
            if value <= 0:
                raise ValueError(f"{name} must be > 0")
        for name in (
            "step_size",
            "collision_step",
            "goal_reach_tolerance",
            "gamma",
            "max_connection_radius",
            "jit_sample_radius",
            "manipulability_eta",
            "manipulability_epsilon",
        ):
            value = getattr(self, name)
            if not _is_valid_real(value):
                raise TypeError(f"{name} must be a finite real number")
            if value <= 0:
                raise ValueError(f"{name} must be > 0")
        if not _is_valid_real(self.goal_sample_rate):
            raise TypeError("goal_sample_rate must be a finite real number")
        if not 0.0 <= self.goal_sample_rate <= 1.0:
            raise ValueError("goal_sample_rate must be in [0, 1]")
        if not _is_valid_real(self.jit_bias_probability):
            raise TypeError("jit_bias_probability must be a finite real number")
        if not 0.0 <= self.jit_bias_probability < 1.0:
            raise ValueError("jit_bias_probability must be in [0, 1)")
        if not _is_valid_real(self.manipulability_weight):
            raise TypeError("manipulability_weight must be a finite real number")
        if self.manipulability_weight < 0:
            raise ValueError("manipulability_weight must be >= 0")
        if self.time_budget_s is not None:
            if not _is_valid_real(self.time_budget_s):
                raise TypeError("time_budget_s must be a finite real number or None")
            if self.time_budget_s <= 0:
                raise ValueError("time_budget_s must be > 0 when provided")
        return self


@dataclass(slots=True)
class HybridAStarParams(Mapping[str, object]):
    """Search limits and discretization for Hybrid A*."""

    max_expansions: int = 100_000
    time_budget_s: float | None = None
    xy_resolution: float | None = None
    heading_bins: int = 72
    primitive_length: float | None = None
    collision_step: float | None = None
    goal_xy_tolerance: float = 0.5
    goal_yaw_tolerance: float = 0.35
    analytic_expansion_distance: float = 5.0
    analytic_expansion_interval: int = 5
    heuristic_weight: float = 1.0
    allow_reverse: bool = True
    reverse_penalty: float = 2.0
    steering_penalty: float = 0.1
    direction_switch_penalty: float = 2.0
    allow_python_callbacks: bool = False

    def __getitem__(self, key: str) -> object:
        if key not in self.__dataclass_fields__:
            raise KeyError(key)
        return getattr(self, key)

    def __iter__(self) -> Iterator[str]:
        return iter(self.__dataclass_fields__)

    def __len__(self) -> int:
        return len(self.__dataclass_fields__)

    def validate(self) -> HybridAStarParams:
        """Validate native-search options."""
        if type(self.allow_reverse) is not bool or type(self.allow_python_callbacks) is not bool:
            raise TypeError("allow_reverse and allow_python_callbacks must be bools")
        for name in ("max_expansions", "heading_bins", "analytic_expansion_interval"):
            value = getattr(self, name)
            if isinstance(value, bool) or type(value) is not int:
                raise TypeError(f"{name} must be an integer")
            minimum = 8 if name == "heading_bins" else 1
            if value < minimum:
                raise ValueError(f"{name} must be >= {minimum}")
        for name in (
            "xy_resolution",
            "primitive_length",
            "collision_step",
            "goal_xy_tolerance",
            "goal_yaw_tolerance",
            "analytic_expansion_distance",
            "heuristic_weight",
            "reverse_penalty",
            "steering_penalty",
            "direction_switch_penalty",
        ):
            value = getattr(self, name)
            if value is None:
                continue
            if not _is_valid_real(value):
                raise TypeError(f"{name} must be a finite real number or None")
            if name == "analytic_expansion_distance":
                if value < 0:
                    raise ValueError(f"{name} must be >= 0")
            elif name in ("steering_penalty", "direction_switch_penalty"):
                if value < 0:
                    raise ValueError(f"{name} must be >= 0")
            elif name == "heuristic_weight" or name == "reverse_penalty":
                if value < 1:
                    raise ValueError(f"{name} must be >= 1")
            elif value <= 0:
                raise ValueError(f"{name} must be > 0")
        if self.time_budget_s is not None:
            if not _is_valid_real(self.time_budget_s):
                raise TypeError("time_budget_s must be a finite real number or None")
            if self.time_budget_s <= 0:
                raise ValueError("time_budget_s must be > 0 when provided")
        return self


@dataclass(slots=True)
class StateLatticeParams(Mapping[str, object]):
    """Search limits and discretization for state-lattice planning."""

    max_expansions: int = 100_000
    time_budget_s: float | None = None
    xy_resolution: float | None = None
    heading_bins: int = 72
    collision_step: float | None = None
    goal_xy_tolerance: float = 0.25
    goal_yaw_tolerance: float = 0.2
    heuristic_weight: float = 1.0
    reverse_penalty: float = 1.0
    direction_switch_penalty: float = 0.0
    allow_python_callbacks: bool = False

    def __getitem__(self, key: str) -> object:
        if key not in self.__dataclass_fields__:
            raise KeyError(key)
        return getattr(self, key)

    def __iter__(self) -> Iterator[str]:
        return iter(self.__dataclass_fields__)

    def __len__(self) -> int:
        return len(self.__dataclass_fields__)

    def validate(self) -> StateLatticeParams:
        """Validate native state-lattice options."""
        if type(self.allow_python_callbacks) is not bool:
            raise TypeError("allow_python_callbacks must be a bool")
        for name in ("max_expansions", "heading_bins"):
            value = getattr(self, name)
            if isinstance(value, bool) or type(value) is not int:
                raise TypeError(f"{name} must be an integer")
            minimum = 8 if name == "heading_bins" else 1
            if value < minimum:
                raise ValueError(f"{name} must be >= {minimum}")
        for name in (
            "xy_resolution",
            "collision_step",
            "goal_xy_tolerance",
            "goal_yaw_tolerance",
            "heuristic_weight",
            "reverse_penalty",
            "direction_switch_penalty",
        ):
            value = getattr(self, name)
            if value is None:
                if name in ("xy_resolution", "collision_step"):
                    continue
                raise TypeError(f"{name} must be a finite real number")
            if not _is_valid_real(value):
                raise TypeError(f"{name} must be a finite real number")
            if name == "direction_switch_penalty":
                if value < 0:
                    raise ValueError(f"{name} must be >= 0")
            elif name in ("heuristic_weight", "reverse_penalty"):
                if value < 1:
                    raise ValueError(f"{name} must be >= 1")
            elif value <= 0:
                raise ValueError(f"{name} must be > 0")
        if self.time_budget_s is not None:
            if not _is_valid_real(self.time_budget_s):
                raise TypeError("time_budget_s must be a finite real number or None")
            if self.time_budget_s <= 0:
                raise ValueError("time_budget_s must be > 0 when provided")
        return self
