"""Deprecated compatibility shim for legacy module name."""

from __future__ import annotations

import warnings

warnings.warn("BIT_star3D is deprecated; use bit_star_3d.", DeprecationWarning, stacklevel=2)

from .bit_star_3d import *  # noqa: F401,F403
