"""Deprecated compatibility shim for legacy module name."""

from __future__ import annotations

import warnings

warnings.warn("extend_rrt3D is deprecated; use extend_rrt_3d.", DeprecationWarning, stacklevel=2)

from .extend_rrt_3d import *  # noqa: F401,F403
