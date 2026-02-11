"""Deprecated compatibility shim for legacy module name."""

from __future__ import annotations

import warnings

warnings.warn("env3D is deprecated; use env_3d.", DeprecationWarning, stacklevel=2)

from .env_3d import *  # noqa: F401,F403
