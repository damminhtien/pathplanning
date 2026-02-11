"""Deprecated compatibility shim for legacy module name."""

from __future__ import annotations

import warnings

warnings.warn("Anytime_Dstar3D is deprecated; use anytime_dstar_3d.", DeprecationWarning, stacklevel=2)

from .anytime_dstar_3d import *  # noqa: F401,F403
