"""Deprecated compatibility shim for legacy module name."""

from __future__ import annotations

import warnings

warnings.warn("dynamic_rrt3D is deprecated; use dynamic_rrt_3d.", DeprecationWarning, stacklevel=2)

from .dynamic_rrt_3d import *  # noqa: F401,F403
