"""Lazy import helpers for optional visualization dependencies."""

from __future__ import annotations

import importlib
from types import ModuleType
from typing import Any


class _LazyModule:
    """Simple proxy that imports a module on first attribute access."""

    def __init__(self, module_name: str) -> None:
        self._module_name = module_name
        self._module: ModuleType | None = None

    def _load(self) -> ModuleType:
        if self._module is None:
            self._module = importlib.import_module(self._module_name)
        return self._module

    def __getattr__(self, name: str) -> Any:
        return getattr(self._load(), name)


def lazy_import(module_name: str) -> Any:
    """Return a module proxy that defers import until first use."""

    return _LazyModule(module_name)
