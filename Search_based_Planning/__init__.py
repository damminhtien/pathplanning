"""Compatibility shim; code now lives in pathplanning.Search_based_Planning."""

from importlib import import_module
import sys

_impl = import_module("pathplanning.Search_based_Planning")
sys.modules[__name__] = _impl
