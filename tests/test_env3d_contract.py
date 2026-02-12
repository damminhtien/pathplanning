"""Environment3D API contract tests."""

from __future__ import annotations

from pathplanning.spaces.environment3d import Environment3D


def test_environment3d_exposes_canonical_snake_case_fields() -> None:
    """The canonical obstacle fields should be snake_case and populated."""
    env = Environment3D()
    assert env.aabb
    assert env.aabb_pyrr
    assert env.obb is not None
    assert env.blocks.shape[1] == 6


def test_environment3d_does_not_expose_legacy_uppercase_fields() -> None:
    """Legacy uppercase environment aliases should be removed."""
    env = Environment3D()
    assert not hasattr(env, "AABB")
    assert not hasattr(env, "AABB_pyrr")
    assert not hasattr(env, "OBB")
