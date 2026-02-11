"""Deprecated compatibility shim for legacy module name."""

from __future__ import annotations

import warnings

warnings.warn("plot_util3D is deprecated; use plot_util_3d.", DeprecationWarning, stacklevel=2)

from .plot_util_3d import *  # noqa: F401,F403
