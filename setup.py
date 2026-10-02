"""Build configuration for the native C and C++ pathplanning core."""

from __future__ import annotations

from setuptools import Extension, setup

setup(
    ext_modules=[
        Extension(
            "pathplanning.native._search_engine",
            sources=["pathplanning/native/search_engine.cpp"],
            include_dirs=["pathplanning/native"],
            language="c++",
            extra_compile_args=["-std=c++17", "-O3"],
        ),
        Extension(
            "pathplanning.native._continuous_engine",
            sources=["pathplanning/native/continuous_engine.c"],
            include_dirs=["pathplanning/native"],
            language="c",
            extra_compile_args=["-std=c11", "-O3"],
        ),
    ],
)
