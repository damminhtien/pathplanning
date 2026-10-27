"""Architecture invariants for package layering and planner structure."""

from __future__ import annotations

import ast
from pathlib import Path


def _iter_python_files(root: Path) -> list[Path]:
    return sorted(path for path in root.rglob("*.py") if path.name != "__init__.py")


def test_planner_modules_do_not_import_viz_matplotlib_or_examples() -> None:
    planners_root = Path("pathplanning/planners")
    banned_prefixes = ("matplotlib", "pathplanning.viz", "examples", "scripts")
    offenders: list[str] = []

    for file_path in _iter_python_files(planners_root):
        source = file_path.read_text(encoding="utf-8")
        tree = ast.parse(source)
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                for alias in node.names:
                    if alias.name.startswith(banned_prefixes):
                        offenders.append(f"{file_path.as_posix()}:{node.lineno}:{alias.name}")
            elif isinstance(node, ast.ImportFrom) and node.module is not None:
                if node.module.startswith(banned_prefixes):
                    offenders.append(f"{file_path.as_posix()}:{node.lineno}:{node.module}")

    assert not offenders, (
        "Planner modules must not import viz/matplotlib/examples/scripts:\n" + "\n".join(offenders)
    )
