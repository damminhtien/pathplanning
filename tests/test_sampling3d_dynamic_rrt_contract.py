"""Contract tests for the production DynamicRRT3D planner."""

from __future__ import annotations

import numpy as np
import pytest

from pathplanning.planners.sampling.dynamic_rrt import (
    DynamicRRT3D,
    DynamicRRT3DConfig,
)


class _CallbackSpace3D:
    def __init__(self) -> None:
        self.sample_calls = 0

    def sample_free(self, rng: np.random.Generator) -> np.ndarray:
        self.sample_calls += 1
        return rng.uniform(0.0, 1.0, size=3)

    def is_state_valid(self, _state: np.ndarray) -> bool:
        return True

    def is_motion_valid(self, _start: np.ndarray, _end: np.ndarray) -> bool:
        return True

    def distance(self, start: np.ndarray, end: np.ndarray) -> float:
        return float(np.linalg.norm(end - start))

    def steer(self, start: np.ndarray, target: np.ndarray, step_size: float) -> np.ndarray:
        delta = target - start
        distance = float(np.linalg.norm(delta))
        if distance <= step_size:
            return target
        return start + delta * (step_size / distance)


def test_dynamic_rrt_choose_target_is_deterministic_with_seed() -> None:
    """Seeded planners should generate the same sampled target sequence."""
    planner_a = DynamicRRT3D.with_seed(7)
    planner_b = DynamicRRT3D.with_seed(7)

    planner_a.init_rrt()
    planner_b.init_rrt()

    shared_nodes = [(3.0, 3.0, 1.0), (4.0, 9.0, 1.0), (8.0, 5.0, 2.0)]
    for node in shared_nodes:
        planner_a.add_node(planner_a.x0, node)
        planner_b.add_node(planner_b.x0, node)

    sequence_a = [planner_a.choose_target() for _ in range(25)]
    sequence_b = [planner_b.choose_target() for _ in range(25)]

    assert sequence_a == sequence_b


def test_dynamic_rrt_custom_space_requires_callback_opt_in() -> None:
    planner = DynamicRRT3D(
        environment=_CallbackSpace3D(),
        config=DynamicRRT3DConfig(max_iterations=0),
        start=[0.0, 0.0, 0.0],
        goal=[1.0, 1.0, 1.0],
    )
    planner.init_rrt()

    with pytest.raises(ValueError, match="requires a built-in or declared native space model"):
        planner.grow_rrt()


def test_dynamic_rrt_custom_space_callbacks_can_be_explicitly_enabled() -> None:
    space = _CallbackSpace3D()
    planner = DynamicRRT3D(
        environment=space,
        config=DynamicRRT3DConfig(
            max_iterations=0,
            allow_python_callbacks=True,
            goal_sample_probability=0.0,
            way_point_sample_probability=0.0,
        ),
        start=[0.0, 0.0, 0.0],
        goal=[1.0, 1.0, 1.0],
        rng=np.random.default_rng(4),
    )
    planner.init_rrt()

    planner.grow_rrt()

    assert planner.nodes[0] == (0.0, 0.0, 0.0)
    assert space.sample_calls > 0
