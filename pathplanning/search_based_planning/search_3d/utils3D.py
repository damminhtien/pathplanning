"""Deprecated compatibility shim for legacy module name."""

from __future__ import annotations

import warnings

warnings.warn("utils3D is deprecated; use utils_3d.", DeprecationWarning, stacklevel=2)

from .utils_3d import *  # noqa: F401,F403
