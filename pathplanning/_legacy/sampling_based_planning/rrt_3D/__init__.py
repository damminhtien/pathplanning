"""Deprecated compatibility shim for ``pathplanning.sampling_based_planning.rrt_3D``."""

from __future__ import annotations

import warnings
from importlib import import_module

warnings.warn(
    "pathplanning.sampling_based_planning.rrt_3D is deprecated; use "
    "pathplanning.sampling_based_planning.rrt_3d.",
    DeprecationWarning,
    stacklevel=2,
)

_impl = import_module("pathplanning.sampling_based_planning.rrt_3d")
__path__ = _impl.__path__
__all__ = list(getattr(_impl, "__all__", []))

if __all__:
    globals().update({name: getattr(_impl, name) for name in __all__})
else:
    globals().update({name: getattr(_impl, name) for name in dir(_impl) if not name.startswith("_")})
