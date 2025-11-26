"""Fail-fast architecture guards for planner-layer cleanup."""

from __future__ import annotations

import ast
import importlib.util
from pathlib import Path
import subprocess
import sys

PACKAGE_ROOT = Path("pathplanning")
PLANNERS_ROOT = PACKAGE_ROOT / "planners"
SEARCH_ROOT = PLANNERS_ROOT / "search"


def _iter_planner_sources() -> list[Path]:
    return sorted(path for path in PLANNERS_ROOT.rglob("*.py"))


def _find_spec(module_name: str):
    try:
        return importlib.util.find_spec(module_name)
    except ModuleNotFoundError:
        return None


def test_planners_do_not_import_matplotlib_or_viz() -> None:
    """Planner layer must stay headless and not depend on viz/matplotlib."""
    offenders: list[str] = []
    for file_path in _iter_planner_sources():
        source = file_path.read_text(encoding="utf-8")
        tree = ast.parse(source)
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                for alias in node.names:
                    if (
                        alias.name == "matplotlib"
                        or alias.name.startswith("matplotlib.")
                        or alias.name == "pathplanning.viz"
                        or alias.name.startswith("pathplanning.viz.")
                    ):
                        offenders.append(f"{file_path.as_posix()}:{node.lineno}:{alias.name}")
            elif isinstance(node, ast.ImportFrom) and node.module is not None:
                if (
                    node.module == "matplotlib"
                    or node.module.startswith("matplotlib.")
                    or node.module == "pathplanning.viz"
                    or node.module.startswith("pathplanning.viz.")
                ):
                    offenders.append(f"{file_path.as_posix()}:{node.lineno}:{node.module}")

    assert not offenders, (
        "Planner modules must not import matplotlib or pathplanning.viz directly:\n"
        + "\n".join(offenders)
    )


def test_legacy_contract_symbols_are_removed() -> None:
    """Core must not expose compatibility-only legacy symbols."""
    from pathplanning.core import contracts

    config_space_name = "Configuration" + "Space"
    batch_config_space_name = "Batch" + "Configuration" + "Space"
    assert not hasattr(contracts, config_space_name)
    assert not hasattr(contracts, batch_config_space_name)


def test_legacy_search_file_patterns_are_removed() -> None:
    """Search planners must stay dimension-agnostic and legacy-wrapper free."""
    offenders = sorted(
        [
            path.as_posix()
            for path in SEARCH_ROOT.glob("*_3d.py")
            if path.is_file()
        ]
        + [
            path.as_posix()
            for path in SEARCH_ROOT.glob("_legacy*.py")
            if path.is_file()
        ]
    )
    explicit_legacy = [
        PACKAGE_ROOT / "search2d.py",
        SEARCH_ROOT / "plan2d_facade.py",
    ]
    offenders.extend(path.as_posix() for path in explicit_legacy if path.exists())
    assert not offenders, "Legacy search files must stay absent:\n" + "\n".join(offenders)


def test_legacy_sampling_wrappers_and_utils_are_removed() -> None:
    """Legacy sampling wrappers/helpers must stay absent after cleanup."""
    banned = [
        PLANNERS_ROOT / "sampling" / "rrt_grid2d.py",
        PACKAGE_ROOT / "utils" / "sampling2d.py",
        PACKAGE_ROOT / "utils" / "sampling3d.py",
    ]
    offenders = [path.as_posix() for path in banned if path.exists()]
    assert not offenders, (
        "Legacy sampling wrappers/helpers must stay absent:\n" + "\n".join(offenders)
    )


def test_removed_legacy_modules_stay_absent() -> None:
    module_names = [
        "pathplanning.search2d",
        "pathplanning.planners.search.entrypoints",
        "pathplanning.planners.sampling.entrypoints",
        "pathplanning.planners.sampling.rrt_grid2d",
        "pathplanning.planners.search._legacy2d_common",
        "pathplanning.planners.search.plan2d_facade",
    ]
    offenders = [module_name for module_name in module_names if _find_spec(module_name) is not None]
    assert not offenders, "Legacy modules must stay absent:\n" + "\n".join(offenders)


def test_importing_public_api_does_not_load_matplotlib() -> None:
    """Importing pathplanning.api must not trigger matplotlib import."""
    code = (
        "import sys\n"
        "before = set(sys.modules)\n"
        "import pathplanning.api\n"
        "loaded = set(sys.modules) - before\n"
        "bad = sorted(\n"
        "    name\n"
        "    for name in loaded\n"
        "    if name == 'matplotlib' or name.startswith('matplotlib.')\n"
        ")\n"
        "if bad:\n"
        "    raise SystemExit('\\n'.join(bad))\n"
    )
    result = subprocess.run(
        [sys.executable, "-c", code],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, (
        "Importing pathplanning.api unexpectedly loaded matplotlib:\n"
        f"{result.stdout}{result.stderr}"
    )


def test_sampling_registry_contains_expected_algorithms() -> None:
    """Sampling registry should expose the full supported planner set."""
    from pathplanning.registry import SAMPLING_PLANNERS

    expected = {
        "rrt",
        "rrt_star",
        "informed_rrt_star",
        "bit_star",
        "fmt_star",
        "rrt_connect",
        "abit_star",
    }
    missing = sorted(expected.difference(SAMPLING_PLANNERS))
    assert not missing, "Sampling registry is missing supported algorithms:\n" + "\n".join(missing)
