"""Build configuration for the native C and C++ pathplanning core."""

from __future__ import annotations

from pathlib import Path

from setuptools import Extension, setup
from setuptools.command.build_ext import build_ext


class IsolatedNativeBuildExt(build_ext):
    """Keep production and diagnostic objects separate despite shared sources."""

    def build_extension(self, ext: Extension) -> None:
        original_build_temp = self.build_temp
        extension_name = ext.name.rsplit(".", 1)[-1]
        self.build_temp = str(Path(original_build_temp) / extension_name)
        try:
            super().build_extension(ext)
        finally:
            self.build_temp = original_build_temp


setup(
    cmdclass={"build_ext": IsolatedNativeBuildExt},
    ext_modules=[
        Extension(
            "pathplanning.native._search_engine",
            sources=[
                "pathplanning/native/search_engine.cpp",
                "pathplanning/native/jps_grid.cpp",
            ],
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
        Extension(
            "pathplanning.native._search_trace_engine",
            sources=[
                "pathplanning/native/search_engine.cpp",
                "pathplanning/native/jps_grid.cpp",
            ],
            include_dirs=["pathplanning/native"],
            language="c++",
            define_macros=[("PP_ENABLE_TRACE", "1")],
            extra_compile_args=["-std=c++17", "-O3"],
        ),
        Extension(
            "pathplanning.native._search_metrics_engine",
            sources=["pathplanning/native/search_engine.cpp"],
            include_dirs=["pathplanning/native"],
            language="c++",
            define_macros=[("PP_ENABLE_METRICS", "1")],
            extra_compile_args=["-std=c++17", "-O3"],
        ),
        Extension(
            "pathplanning.native._continuous_trace_engine",
            sources=["pathplanning/native/continuous_engine.c"],
            include_dirs=["pathplanning/native"],
            language="c",
            define_macros=[("PP_ENABLE_TRACE", "1")],
            extra_compile_args=["-std=c11", "-O3"],
        ),
    ],
)
