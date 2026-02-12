"""Determinism guardrails for sampling-based planners."""

from __future__ import annotations

import importlib
import inspect
from pathlib import Path

import numpy as np

from pathplanning.core.params import RrtParams
from pathplanning.planners.sampling.rrt import RrtPlanner
from pathplanning.planners.sampling.rrt_star import RrtStarPlanner
from pathplanning.registry import expected_entrypoint_for_algorithm, list_supported_algorithms
from pathplanning.spaces.continuous_3d import AABB, ContinuousSpace3D


def test_sampling_planners_accept_rng() -> None:
    """All supported sampling planners should accept an injected RNG."""
    for spec in list_supported_algorithms():
        if spec.family != "sampling":
            continue
        module = importlib.import_module(spec.module)
        entrypoint = expected_entrypoint_for_algorithm(spec.algorithm_id)
        planner_cls = getattr(module, entrypoint)
        signature = inspect.signature(planner_cls.__init__)
        assert "rng" in signature.parameters, (
            f"{spec.algorithm_id} entrypoint '{entrypoint}' must accept rng parameter"
        )


def test_supported_sampling_modules_do_not_use_global_np_random() -> None:
    """Supported sampling modules should not call global np.random helpers."""
    forbidden = (
        "np.random.random",
        "np.random.default_rng",
        "np.random.uniform",
        "np.random.randint",
        "np.random.choice",
        "np.random.rand",
    )
    for spec in list_supported_algorithms():
        if spec.family != "sampling":
            continue
        module = importlib.import_module(spec.module)
        source = Path(module.__file__).read_text(encoding="utf-8")
        for token in forbidden:
            assert token not in source, f"{spec.module} uses forbidden global RNG call: {token}"


def test_planners_do_not_use_global_np_random(monkeypatch) -> None:
    """Injected RNG should be the only randomness source in core planners."""

    def _raise(*_args: object, **_kwargs: object) -> None:
        raise AssertionError("global np.random usage is forbidden in planners")

    rng_rrt = np.random.default_rng(1234)
    rng_rrt_star = np.random.default_rng(4321)
    for name in ("random", "default_rng", "uniform", "randint", "choice", "rand"):
        monkeypatch.setattr(np.random, name, _raise, raising=True)

    space = ContinuousSpace3D(
        lower_bound=[0.0, 0.0, 0.0],
        upper_bound=[5.0, 5.0, 5.0],
        aabbs=[AABB([2.0, 2.0, 0.0], [3.0, 3.0, 5.0])],
    )
    params = RrtParams(max_iters=10, step_size=0.5, goal_sample_rate=0.2)

    RrtPlanner(space, params, rng_rrt).plan([0.5, 0.5, 0.5], ([4.5, 4.5, 0.5], 0.5))
    RrtStarPlanner(space, params, rng_rrt_star).plan([0.5, 0.5, 0.5], ([4.5, 4.5, 0.5], 0.5))
