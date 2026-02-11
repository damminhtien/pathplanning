"""Environment3D API contract and migration tests."""

from __future__ import annotations

import warnings

from pathplanning.sampling_based_planning.rrt_3d.env_3d import Environment3D


def test_environment3d_exposes_canonical_snake_case_fields() -> None:
    """The canonical obstacle fields should be snake_case and populated."""
    env = Environment3D()
    assert env.aabb
    assert env.aabb_pyrr
    assert env.obb is not None
    assert env.blocks.shape[1] == 6


def test_environment3d_legacy_fields_emit_deprecation_warnings() -> None:
    """Legacy uppercase environment fields remain available with deprecation warnings."""
    env = Environment3D()
    with warnings.catch_warnings(record=True) as captured:
        warnings.simplefilter("always", DeprecationWarning)
        _ = env.AABB
        _ = env.AABB_pyrr
        _ = env.OBB

    warning_messages = [str(item.message) for item in captured]
    assert any("AABB" in message for message in warning_messages)
    assert any("AABB_pyrr" in message for message in warning_messages)
    assert any("OBB" in message for message in warning_messages)
