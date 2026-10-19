"""Run strict syntax and warning checks for every native source and header."""

from __future__ import annotations

import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
NATIVE_ROOT = ROOT / "pathplanning" / "native"
COMMON_FLAGS = ("-Wall", "-Wextra", "-Wpedantic", "-Werror", "-fsyntax-only")
C_MODES = ((), ("PP_ENABLE_TRACE",))
CPP_MODES = ((), ("PP_ENABLE_TRACE",), ("PP_ENABLE_METRICS",))
C_SOURCE_SUFFIXES = {".c"}
CPP_SOURCE_SUFFIXES = {".cc", ".cpp", ".cxx"}
C_HEADER_SUFFIXES = {".h"}
CPP_HEADER_SUFFIXES = {".hh", ".hpp", ".hxx"}


def compiler_command(variable: str, fallback: str) -> list[str] | None:
    """Resolve a compiler command, allowing conventional compiler flags in env vars."""
    command = shlex.split(os.environ.get(variable, fallback))
    if not command or shutil.which(command[0]) is None:
        return None
    return command


def lint_one(
    compiler: list[str],
    source: Path,
    *,
    language: str,
    standard: str,
    defines: tuple[str, ...],
) -> tuple[str, str] | None:
    command = [
        *compiler,
        f"-std={standard}",
        *COMMON_FLAGS,
        "-I",
        str(NATIVE_ROOT),
        *(f"-D{define}" for define in defines),
    ]
    if source.suffix in C_HEADER_SUFFIXES | CPP_HEADER_SUFFIXES:
        command.extend(("-x", language, "-include", str(source), "-"))
        source_text = "int pathplanning_lint_translation_unit(void) { return 0; }\n"
    else:
        command.append(str(source))
        source_text = None

    result = subprocess.run(
        command,
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
        input=source_text,
    )
    if result.returncode == 0:
        return None

    mode = ", ".join(defines) if defines else "default"
    detail = result.stderr.strip() or result.stdout.strip()
    return f"{source.relative_to(ROOT)} [{standard}, {mode}]", detail


def main() -> int:
    c_compiler = compiler_command("CC", "cc")
    cpp_compiler = compiler_command("CXX", "c++")
    if c_compiler is None or cpp_compiler is None:
        print("Native lint requires C11 and C++17 compilers (CC and CXX).", file=sys.stderr)
        return 1

    checks: list[tuple[list[str], Path, str, str, tuple[str, ...]]] = []
    native_files = [path for path in NATIVE_ROOT.rglob("*") if path.is_file()]
    c_sources = sorted(path for path in native_files if path.suffix in C_SOURCE_SUFFIXES)
    cpp_sources = sorted(path for path in native_files if path.suffix in CPP_SOURCE_SUFFIXES)
    c_headers = sorted(path for path in native_files if path.suffix in C_HEADER_SUFFIXES)
    cpp_headers = sorted(path for path in native_files if path.suffix in CPP_HEADER_SUFFIXES)

    for source in c_sources:
        checks.extend((c_compiler, source, "c", "c11", mode) for mode in C_MODES)
    for source in cpp_sources:
        checks.extend((cpp_compiler, source, "c++", "c++17", mode) for mode in CPP_MODES)
    for header in c_headers:
        checks.extend((c_compiler, header, "c", "c11", mode) for mode in C_MODES)
        checks.extend((cpp_compiler, header, "c++", "c++17", mode) for mode in CPP_MODES)
    for header in cpp_headers:
        checks.extend((cpp_compiler, header, "c++", "c++17", mode) for mode in CPP_MODES)

    failures = [
        failure
        for compiler, source, language, standard, defines in checks
        if (
            failure := lint_one(
                compiler,
                source,
                language=language,
                standard=standard,
                defines=defines,
            )
        )
        is not None
    ]
    if failures:
        for target, detail in failures:
            print(f"Native lint failed: {target}\n{detail}", file=sys.stderr)
        return 1

    print(
        f"Native lint passed: {len(c_sources)} C sources, {len(cpp_sources)} C++ sources, "
        f"{len(c_headers)} C headers, and {len(cpp_headers)} C++ headers "
        f"({len(checks)} strict compiler configurations)."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
