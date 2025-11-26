"""Shared adapters for problem-oriented sampling planner wrappers."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import fields

import numpy as np

from pathplanning.core.contracts import ContinuousProblem, State
from pathplanning.core.params import RrtParams
from pathplanning.core.types import RNG

_ALLOWED_RRT_PARAM_KEYS = {field.name for field in fields(RrtParams)}


def coerce_rrt_params(
    problem: ContinuousProblem[State],
    params: RrtParams | Mapping[str, object] | None,
) -> RrtParams:
    """Resolve ``RrtParams`` from problem defaults and call overrides."""
    if isinstance(params, RrtParams):
        return params.validate()

    problem_params = problem.params
    merged_kwargs: dict[str, object] = {}

    if problem_params is not None:
        invalid = set(problem_params) - _ALLOWED_RRT_PARAM_KEYS
        if invalid:
            invalid_values = ", ".join(sorted(invalid))
            raise KeyError(f"Unsupported continuous problem params: {invalid_values}")
        merged_kwargs.update(problem_params)

    if params is not None:
        invalid = set(params) - _ALLOWED_RRT_PARAM_KEYS
        if invalid:
            invalid_values = ", ".join(sorted(invalid))
            raise KeyError(f"Unsupported planner params override: {invalid_values}")
        merged_kwargs.update(params)

    return RrtParams(**merged_kwargs)


def resolve_rng(rng: RNG | None) -> RNG:
    """Return caller RNG or one deterministic default generator."""
    return rng if rng is not None else np.random.default_rng(0)


__all__ = ["coerce_rrt_params", "resolve_rng"]
