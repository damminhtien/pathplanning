from __future__ import annotations

import numpy as np
import pytest

from pathplanning.spaces.continuous_nd import backtrack_path_edges_nd, sample_valid_state_nd


def _within_bounds(sample: np.ndarray, lower: np.ndarray, upper: np.ndarray) -> bool:
    return bool(np.all(sample >= lower) and np.all(sample <= upper))


def test_sample_valid_state_nd_within_bounds() -> None:
    rng = np.random.default_rng(7)
    lower = np.array([0.0, 0.0, 0.0], dtype=float)
    upper = np.array([1.0, 1.0, 1.0], dtype=float)

    sample = sample_valid_state_nd(
        rng,
        lower,
        upper,
        is_state_valid=lambda _x: True,
        dim=3,
        max_tries=10,
    )

    assert _within_bounds(sample, lower, upper)


def test_sample_valid_state_nd_returns_bounded_fallback_when_all_invalid() -> None:
    rng = np.random.default_rng(0)
    lower = np.array([0.0, 0.0, 0.0], dtype=float)
    upper = np.array([1.0, 1.0, 1.0], dtype=float)

    sample = sample_valid_state_nd(
        rng,
        lower,
        upper,
        is_state_valid=lambda _x: False,
        dim=3,
        max_tries=3,
    )

    assert _within_bounds(sample, lower, upper)


def test_sample_valid_state_nd_goal_bias_prefers_valid_goal() -> None:
    rng = np.random.default_rng(123)
    goal = np.array([0.8, 0.2, 0.1], dtype=float)

    sample = sample_valid_state_nd(
        rng,
        [0.0, 0.0, 0.0],
        [1.0, 1.0, 1.0],
        is_state_valid=lambda _x: True,
        dim=3,
        max_tries=10,
        goal_state=goal,
        goal_sample_rate=1.0,
    )

    assert np.allclose(sample, goal)


def test_backtrack_path_edges_nd_does_not_accumulate_between_calls() -> None:
    parent_by_node = {(1.0, 0.0, 0.0): (0.0, 0.0, 0.0)}

    first_path, first_dist = backtrack_path_edges_nd(
        start=(0.0, 0.0, 0.0),
        goal=(1.0, 0.0, 0.0),
        parent_by_node=parent_by_node,
        dim=3,
    )
    second_path, second_dist = backtrack_path_edges_nd(
        start=(0.0, 0.0, 0.0),
        goal=(1.0, 0.0, 0.0),
        parent_by_node=parent_by_node,
        dim=3,
    )

    assert first_path is not second_path
    assert len(first_path) == 1
    assert len(second_path) == 1
    assert first_dist == pytest.approx(1.0)
    assert second_dist == pytest.approx(1.0)


def test_backtrack_path_edges_nd_raises_when_parent_chain_missing() -> None:
    with pytest.raises(KeyError):
        backtrack_path_edges_nd(
            start=(0.0, 0.0, 0.0),
            goal=(1.0, 1.0, 1.0),
            parent_by_node={},
            dim=3,
        )
