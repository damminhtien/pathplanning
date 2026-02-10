"""Compatibility shim; code now lives in pathplanning.CurvesGenerator."""

from importlib import import_module
import sys

_impl = import_module("pathplanning.CurvesGenerator")
sys.modules[__name__] = _impl
