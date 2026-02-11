"""Deprecated compatibility shim for legacy module name."""

from __future__ import annotations

import warnings

warnings.warn("ABIT_star3D is deprecated; use abit_star_3d.", DeprecationWarning, stacklevel=2)

from .abit_star_3d import *  # noqa: F401,F403
