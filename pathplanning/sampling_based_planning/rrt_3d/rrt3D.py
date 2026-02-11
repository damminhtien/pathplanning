"""Deprecated compatibility shim for legacy module name."""

from __future__ import annotations

import warnings

warnings.warn("rrt3D is deprecated; use rrt_3d.", DeprecationWarning, stacklevel=2)

from .rrt_3d import *  # noqa: F401,F403
