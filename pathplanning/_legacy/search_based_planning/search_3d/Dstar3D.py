"""Deprecated compatibility shim for legacy module name."""

from __future__ import annotations

import warnings

warnings.warn("Dstar3D is deprecated; use dstar_3d.", DeprecationWarning, stacklevel=2)

from .dstar_3d import *  # noqa: F401,F403
