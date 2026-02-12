"""Integrity tests for registry-declared supported algorithms."""

from __future__ import annotations

import ast
import importlib
import importlib.util
import inspect
from pathlib import Path
import subprocess
import sys

from pathplanning.registry import expected_entrypoint_for_algorithm, list_supported_algorithms


def _module_source_path(module_name: str) -> Path:
    spec = importlib.util.find_spec(module_name)
    assert spec is not None, f"Missing module spec for {module_name}"
    assert spec.origin is not None, f"Missing source origin for {module_name}"
    return Path(spec.origin)


def _is_stub_node(node: ast.AST) -> bool:
    body = list(getattr(node, "body", []))
    if body and isinstance(body[0], ast.Expr):
        value = body[0].value
        if isinstance(value, ast.Constant) and isinstance(value.value, str):
            body = body[1:]

    if not body:
        return True

    if len(body) != 1:
        return False

    stmt = body[0]
    if isinstance(stmt, ast.Pass):
        return True
    if isinstance(stmt, ast.Expr):
        value = stmt.value
        if isinstance(value, ast.Constant) and value.value is Ellipsis:
            return True
    if isinstance(stmt, ast.Raise):
        exc = stmt.exc
        if isinstance(exc, ast.Name) and exc.id == "NotImplementedError":
            return True
        if isinstance(exc, ast.Call) and isinstance(exc.func, ast.Name):
            return exc.func.id == "NotImplementedError"
    return False


def _class_is_stubbed(source_text: str, class_name: str) -> bool:
    tree = ast.parse(source_text)
    for node in tree.body:
        if isinstance(node, ast.ClassDef) and node.name == class_name:
            return _is_stub_node(node)
    return False


def _imports_matplotlib(source_text: str) -> bool:
    tree = ast.parse(source_text)
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for alias in node.names:
                if alias.name == "matplotlib" or alias.name.startswith("matplotlib."):
                    return True
        elif (
            isinstance(node, ast.ImportFrom)
            and node.module
            and (node.module == "matplotlib" or node.module.startswith("matplotlib."))
        ):
            return True
    return False


def test_supported_registry_modules_are_usable_planners() -> None:
    """
    Registry integrity contract:
    - every SUPPORTED algorithm module resolves to source and is non-empty
    - module can be imported
    - expected entrypoint exists and is a class
    - planner class entrypoint is not stubbed
    - supported modules do not import matplotlib directly
    """
    for algo_spec in list_supported_algorithms():
        source_path = _module_source_path(algo_spec.module)
        source_text = source_path.read_text(encoding="utf-8")
        assert source_text.strip(), (
            f"{algo_spec.algorithm_id} points to an empty module: {source_path}"
        )
        assert not _imports_matplotlib(source_text), (
            f"{algo_spec.algorithm_id} imports matplotlib in module source: {source_path}"
        )

        expected_entrypoint = expected_entrypoint_for_algorithm(algo_spec.algorithm_id)
        module = importlib.import_module(algo_spec.module)
        assert hasattr(module, expected_entrypoint), (
            f"{algo_spec.algorithm_id} missing expected entrypoint '{expected_entrypoint}' "
            f"in module {algo_spec.module}"
        )
        entrypoint_cls = getattr(module, expected_entrypoint)
        assert inspect.isclass(entrypoint_cls), (
            f"{algo_spec.algorithm_id} entrypoint '{expected_entrypoint}' is not a class"
        )
        assert not _class_is_stubbed(source_text, expected_entrypoint), (
            f"{algo_spec.algorithm_id} entrypoint '{expected_entrypoint}' is stubbed"
        )


def test_importing_all_supported_registry_modules_is_headless() -> None:
    """Importing all supported modules must not load matplotlib."""
    modules = sorted({spec.module for spec in list_supported_algorithms()})
    code = (
        "import importlib\n"
        "import sys\n"
        f"modules = {modules!r}\n"
        "before = set(sys.modules)\n"
        "for module_name in modules:\n"
        "    importlib.import_module(module_name)\n"
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
        [sys.executable, "-c", code], capture_output=True, text=True, check=False
    )
    assert result.returncode == 0, (
        f"Supported registry modules load matplotlib during import:\n{result.stdout}{result.stderr}"
    )
