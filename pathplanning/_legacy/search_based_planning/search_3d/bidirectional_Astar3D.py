"""Deprecated compatibility shim for legacy module name."""

from __future__ import annotations

import warnings

warnings.warn("bidirectional_Astar3D is deprecated; use bidirectional_astar_3d.", DeprecationWarning, stacklevel=2)

from .bidirectional_astar_3d import *  # noqa: F401,F403
