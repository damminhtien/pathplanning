"""Enforce snake_case naming under the pathplanning package."""

from __future__ import annotations

from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "pathplanning"
LEGACY_ROOT = PACKAGE_ROOT / "_legacy"
EXEMPT_MODULE_FILES = {"__init__.py", "__main__.py"}


def _contains_uppercase(value: str) -> bool:
    return any(character.isupper() for character in value)


def _is_legacy_exception(path: Path) -> bool:
    if not LEGACY_ROOT.exists():
        return False
    try:
        path.relative_to(LEGACY_ROOT)
        return True
    except ValueError:
        return False


def test_pathplanning_directories_are_snake_case() -> None:
    violations: list[str] = []
    for directory in PACKAGE_ROOT.rglob("*"):
        if not directory.is_dir():
            continue
        if directory == PACKAGE_ROOT:
            continue
        if _is_legacy_exception(directory):
            continue
        if _contains_uppercase(directory.name):
            violations.append(str(directory.relative_to(PACKAGE_ROOT)))

    assert not violations, "Uppercase directories are not allowed:\n" + "\n".join(
        sorted(violations)
    )


def test_pathplanning_module_files_are_snake_case() -> None:
    violations: list[str] = []
    for module_file in PACKAGE_ROOT.rglob("*.py"):
        if _is_legacy_exception(module_file):
            continue
        if module_file.name in EXEMPT_MODULE_FILES:
            continue
        if _contains_uppercase(module_file.name):
            violations.append(str(module_file.relative_to(PACKAGE_ROOT)))

    assert not violations, "Uppercase Python module files are not allowed:\n" + "\n".join(
        sorted(violations)
    )
