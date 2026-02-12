"""Test configuration ensuring repository root is importable."""

from __future__ import annotations

from pathlib import Path
import sys
import warnings

import pytest

ROOT = Path(__file__).resolve().parents[1]
PROJECT_CODE_ROOT = ROOT / "pathplanning"

if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


def _is_project_warning_filename(filename: str | None) -> bool:
    if not filename or filename.startswith("<"):
        return False

    try:
        warning_path = Path(filename).resolve()
    except (OSError, RuntimeError):  # pragma: no cover - defensive for odd warning filenames
        return False

    return warning_path == PROJECT_CODE_ROOT or PROJECT_CODE_ROOT in warning_path.parents


@pytest.fixture(autouse=True)
def fail_on_project_deprecation_warnings() -> None:
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always", DeprecationWarning)
        warnings.simplefilter("always", PendingDeprecationWarning)
        yield

    project_warnings: list[str] = []
    for warning in caught:
        if not issubclass(warning.category, (DeprecationWarning, PendingDeprecationWarning)):
            continue
        if not _is_project_warning_filename(warning.filename):
            continue
        project_warnings.append(
            f"{warning.filename}:{warning.lineno}: {warning.category.__name__}: {warning.message}"
        )

    if project_warnings:
        details = "\n".join(project_warnings)
        pytest.fail(f"Deprecation warnings from project code are not allowed:\n{details}")
