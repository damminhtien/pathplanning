"""Small, stable public API for planner execution."""

from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
import importlib
from types import ModuleType
from typing import Any, TypeAlias

import numpy as np

from pathplanning.core.contracts import ConfigurationSpace, State
from pathplanning.core.params import RrtParams
from pathplanning.core.results import PlanResult
from pathplanning.registry import expected_entrypoint_for_algorithm, get_algorithm

GoalPredicate: TypeAlias = Callable[[State], bool]
GoalRegion: TypeAlias = GoalPredicate | tuple[Sequence[float], float] | Sequence[float] | State
Result: TypeAlias = PlanResult
Stats: TypeAlias = Mapping[str, float]


def load_algorithm_module(algorithm_id: str) -> ModuleType:
    """Load one planner module by registry algorithm id."""
    return importlib.import_module(get_algorithm(algorithm_id).module)


def _resolve_planner_class(algorithm_id: str) -> type[Any]:
    """Resolve the concrete planner class from the registry."""
    module = load_algorithm_module(algorithm_id)
    entrypoint = expected_entrypoint_for_algorithm(algorithm_id)
    planner_class = getattr(module, entrypoint, None)
    if not isinstance(planner_class, type):
        raise TypeError(
            f"Registry entrypoint '{entrypoint}' for '{algorithm_id}' is not a class"
        )
    return planner_class


def run_planner(
    algorithm_id: str,
    *,
    space: ConfigurationSpace,
    start: Sequence[float] | State,
    goal: GoalRegion,
    params: RrtParams | None = None,
    seed: int | None = 0,
    rng: np.random.Generator | None = None,
) -> Result:
    """
    Construct and run one registered planner with deterministic defaults.

    If ``rng`` is omitted, a local generator is created with ``seed=0`` by
    default for reproducible behavior.
    """
    planner_class = _resolve_planner_class(algorithm_id)
    effective_params = params if params is not None else RrtParams()
    effective_rng = rng if rng is not None else np.random.default_rng(seed)

    planner = planner_class(space, effective_params, effective_rng)
    planner_plan = getattr(planner, "plan", None)
    if not callable(planner_plan):
        raise TypeError(f"Planner '{planner_class.__name__}' does not expose a callable plan()")

    result = planner_plan(start, goal)
    if not isinstance(result, PlanResult):
        raise TypeError(
            f"Planner '{planner_class.__name__}' returned {type(result)!r}; expected PlanResult"
        )
    return result


def plan(
    algorithm_id: str,
    *,
    space: ConfigurationSpace,
    start: Sequence[float] | State,
    goal: GoalRegion,
    params: RrtParams | None = None,
    seed: int | None = 0,
    rng: np.random.Generator | None = None,
) -> Result:
    """Alias for :func:`run_planner`."""
    return run_planner(
        algorithm_id,
        space=space,
        start=start,
        goal=goal,
        params=params,
        seed=seed,
        rng=rng,
    )


__all__ = [
    "GoalRegion",
    "Result",
    "Stats",
    "load_algorithm_module",
    "run_planner",
    "plan",
]
