"""Deprecated compatibility shim for legacy module name."""

from __future__ import annotations

import warnings

warnings.warn("informed_rrt_star3D is deprecated; use informed_rrt_star_3d.", DeprecationWarning, stacklevel=2)

from .informed_rrt_star_3d import *  # noqa: F401,F403
