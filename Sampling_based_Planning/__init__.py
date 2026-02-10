"""Compatibility shim; code now lives in pathplanning.Sampling_based_Planning."""

from importlib import import_module
import sys

_impl = import_module("pathplanning.Sampling_based_Planning")
sys.modules[__name__] = _impl
