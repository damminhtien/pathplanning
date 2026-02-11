"""Deprecated compatibility shim for legacy module name."""

from __future__ import annotations

import warnings

warnings.warn("FMT_star3D is deprecated; use fmt_star_3d.", DeprecationWarning, stacklevel=2)

from .fmt_star_3d import *  # noqa: F401,F403
