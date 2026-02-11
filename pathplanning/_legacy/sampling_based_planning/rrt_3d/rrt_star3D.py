"""Deprecated compatibility shim for legacy module name."""

from __future__ import annotations

import warnings

warnings.warn("rrt_star3D is deprecated; use rrt_star_3d.", DeprecationWarning, stacklevel=2)

from .rrt_star_3d import *  # noqa: F401,F403
