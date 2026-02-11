"""Deprecated compatibility shim for legacy module name."""

from __future__ import annotations

import warnings

warnings.warn("LP_Astar3D is deprecated; use lp_astar_3d.", DeprecationWarning, stacklevel=2)

from .lp_astar_3d import *  # noqa: F401,F403
