"""Environment3D API contract tests."""

from __future__ import annotations

import numpy as np

from pathplanning.spaces.environment3d import Environment3D


def test_environment3d_exposes_canonical_snake_case_fields() -> None:
    """The canonical obstacle fields should be snake_case and shape-safe."""
    env = Environment3D(
        blocks=np.array([[1.0, 1.0, 0.0, 2.0, 2.0, 1.0]], dtype=float),
        balls=np.array([[3.0, 3.0, 0.5, 0.4]], dtype=float),
    )
    assert len(env.aabb) == 1
    assert len(env.aabb_pyrr) == 1
    assert env.obb is not None
    assert env.blocks.shape[1] == 6
    assert env.balls.shape[1] == 4


def test_environment3d_does_not_expose_legacy_uppercase_fields() -> None:
    """Legacy uppercase environment aliases should be removed."""
    env = Environment3D()
    assert not hasattr(env, "AABB")
    assert not hasattr(env, "AABB_pyrr")
    assert not hasattr(env, "OBB")
