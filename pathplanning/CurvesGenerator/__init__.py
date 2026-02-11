"""Deprecated compatibility shim for ``pathplanning.CurvesGenerator``."""

from __future__ import annotations

import warnings
from importlib import import_module

warnings.warn(
    "pathplanning.CurvesGenerator is deprecated; use pathplanning.curves.",
    DeprecationWarning,
    stacklevel=2,
)

_impl = import_module("pathplanning.curves")

# Keep legacy submodule imports working, e.g. pathplanning.CurvesGenerator.dubins_path.
__path__ = _impl.__path__
__all__ = list(getattr(_impl, "__all__", []))

if __all__:
    globals().update({name: getattr(_impl, name) for name in __all__})
else:
    globals().update({name: getattr(_impl, name) for name in dir(_impl) if not name.startswith("_")})
