"""Run the repository's configured Pyright scope against the project Python."""

from __future__ import annotations

import os
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]


def project_python() -> Path:
    """Prefer the repository virtual environment when it exists."""
    candidates = (
        ROOT / ".venv" / "bin" / "python",
        ROOT / ".venv" / "Scripts" / "python.exe",
    )
    return next((path for path in candidates if path.is_file()), Path(sys.executable))


def pyright_command(python: Path) -> list[str] | None:
    """Find Pyright in the selected environment or current PATH."""
    executable_name = "pyright.exe" if os.name == "nt" else "pyright"
    environment_executable = python.parent / executable_name
    if environment_executable.is_file():
        return [str(environment_executable)]

    path_executable = shutil.which("pyright")
    if path_executable:
        return [path_executable]

    probe = subprocess.run(
        [str(python), "-m", "pyright", "--version"],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    if probe.returncode == 0:
        return [str(python), "-m", "pyright"]
    return None


def main() -> int:
    python = project_python()
    command = pyright_command(python)
    if command is None:
        print("Pyright is not installed. Run: pip install -r requirements-dev.txt", file=sys.stderr)
        return 1

    return subprocess.run(
        [
            *command,
            "--project",
            str(ROOT / "pyrightconfig.json"),
            "--pythonpath",
            str(python),
        ],
        cwd=ROOT,
        check=False,
    ).returncode


if __name__ == "__main__":
    raise SystemExit(main())
