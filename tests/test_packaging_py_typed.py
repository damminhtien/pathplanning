"""Packaging tests for typed-distribution markers."""

from __future__ import annotations

from pathlib import Path
import subprocess
import sys
import zipfile


def test_wheel_contains_py_typed_marker(tmp_path: Path) -> None:
    """Build wheel/sdist and verify the wheel ships ``pathplanning/py.typed``."""
    project_root = Path(__file__).resolve().parents[1]
    outdir = tmp_path / "dist"
    outdir.mkdir(parents=True, exist_ok=True)

    subprocess.run(
        [sys.executable, "-m", "build", "--sdist", "--wheel", "--outdir", str(outdir)],
        cwd=project_root,
        check=True,
        capture_output=True,
        text=True,
    )

    wheels = sorted(outdir.glob("*.whl"))
    assert wheels, "Build did not produce a wheel"

    with zipfile.ZipFile(wheels[0]) as wheel_zip:
        wheel_members = set(wheel_zip.namelist())

    assert "pathplanning/py.typed" in wheel_members
