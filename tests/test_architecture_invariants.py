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


def test_planner_modules_do_not_encode_dimensions_in_filenames() -> None:
    planners_root = Path("pathplanning/planners")
    offenders = [
        path.as_posix()
        for path in _iter_python_files(planners_root)
        if path.stem.endswith("_2d") or path.stem.endswith("_3d")
    ]
    assert not offenders, (
        "Planner algorithm files must be dimension-agnostic (no *_2d.py/*_3d.py):\n"
        + "\n".join(offenders)
    )


def test_planner_modules_do_not_use_legacy_wrapper_naming() -> None:
    planners_root = Path("pathplanning/planners")
    offenders = [
        path.as_posix()
        for path in _iter_python_files(planners_root)
        if any(token in path.stem for token in ("legacy", "facade", "wrapper"))
    ]
    assert not offenders, (
        "Legacy compatibility wrappers are not allowed in planners:\n" + "\n".join(offenders)
    )


def test_core_contracts_do_not_expose_legacy_compatibility_interfaces() -> None:
    from pathplanning.core import contracts

    config_space_name = "Configuration" + "Space"
    batch_config_space_name = "Batch" + "Configuration" + "Space"
    assert not hasattr(contracts, config_space_name)
    assert not hasattr(contracts, batch_config_space_name)
