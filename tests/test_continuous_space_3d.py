"""Tests for reusable continuous 3D environment contracts."""

from __future__ import annotations

import numpy as np

from pathplanning.env.continuous_3d import AABB, OBB, ContinuousSpace3D, Sphere


def test_is_free_checks_aabb_sphere_obb_and_bounds() -> None:
    space = ContinuousSpace3D(
        lower_bound=[0.0, 0.0, 0.0],
        upper_bound=[10.0, 10.0, 10.0],
        aabbs=[AABB([2.0, 2.0, 2.0], [4.0, 4.0, 4.0])],
        spheres=[Sphere([8.0, 8.0, 8.0], 1.0)],
        obbs=[OBB([5.0, 5.0, 5.0], [0.5, 0.5, 0.5], np.eye(3, dtype=float))],
    )

    assert space.is_free(np.array([1.0, 1.0, 1.0], dtype=float))
    assert not space.is_free(np.array([3.0, 3.0, 3.0], dtype=float))
    assert not space.is_free(np.array([8.0, 8.0, 8.0], dtype=float))
    assert not space.is_free(np.array([5.0, 5.0, 5.0], dtype=float))
    assert not space.is_free(np.array([11.0, 1.0, 1.0], dtype=float))


def test_segment_free_trivial_and_blocked_cases() -> None:
    free_space = ContinuousSpace3D(
        lower_bound=[0.0, 0.0, 0.0],
        upper_bound=[10.0, 10.0, 10.0],
    )
    assert free_space.segment_free(
        np.array([0.0, 0.0, 0.0], dtype=float),
        np.array([1.0, 1.0, 1.0], dtype=float),
        collision_step=0.1,
    )

    blocked_space = ContinuousSpace3D(
        lower_bound=[0.0, 0.0, 0.0],
        upper_bound=[10.0, 10.0, 10.0],
        aabbs=[AABB([2.0, 2.0, 2.0], [4.0, 4.0, 4.0])],
    )
    assert not blocked_space.segment_free(
        np.array([0.0, 0.0, 0.0], dtype=float),
        np.array([5.0, 5.0, 5.0], dtype=float),
        collision_step=0.1,
    )
