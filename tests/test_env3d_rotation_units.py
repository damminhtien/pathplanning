"""Rotation unit contract for Environment3D helpers."""

from __future__ import annotations

import numpy as np

from pathplanning.spaces.environment3d import rotation_matrix


def test_rotation_matrix_uses_radians_for_angles() -> None:
    """A 90-degree Z rotation should map +X to +Y when converted to radians."""
    rotation = rotation_matrix(np.deg2rad(90.0), 0.0, 0.0)
    x_axis = np.array([1.0, 0.0, 0.0], dtype=float)

    rotated = rotation @ x_axis

    assert np.allclose(rotated, np.array([0.0, 1.0, 0.0], dtype=float), atol=1e-8)
