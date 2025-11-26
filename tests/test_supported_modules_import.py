"""Import and callable contract tests for registered production planners."""

from __future__ import annotations

import ast
import importlib
import importlib.util
from pathlib import Path
import subprocess
import sys

from pathplanning.registry import SAMPLING_PLANNERS, SEARCH_PLANNERS, planner_modules


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


def _callable_is_stubbed(source_text: str, callable_name: str) -> bool:
    tree = ast.parse(source_text)
    for node in tree.body:
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)) and (
            node.name == callable_name
        ):
            return _is_stub_node(node)
    return False


def test_import_all_registered_modules() -> None:
    for module_name in planner_modules():
        importlib.import_module(module_name)


def test_import_registered_modules_does_not_load_matplotlib() -> None:
    for module_name in planner_modules():
        code = (
            "import importlib\n"
            "import sys\n"
            "before = set(sys.modules)\n"
            f"importlib.import_module({module_name!r})\n"
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
            f"{module_name} loads matplotlib during import:\\n{result.stdout}{result.stderr}"
        )


def test_import_all_registered_modules_together_does_not_load_matplotlib() -> None:
    """Importing all planners in one process must stay headless-safe."""
    modules = planner_modules()
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
        f"Batch import of registered modules loads matplotlib:\\n{result.stdout}{result.stderr}"
    )


def test_registered_modules_have_non_empty_source_and_concrete_callables() -> None:
    """Registered planner modules should expose concrete callable implementations."""
    for planner_name, planner_fn in [*SEARCH_PLANNERS.items(), *SAMPLING_PLANNERS.items()]:
        module_name = planner_fn.__module__
        source_path = _module_source_path(module_name)
        source_text = source_path.read_text(encoding="utf-8")
        assert source_text.strip(), f"{planner_name} points to an empty module: {source_path}"

        callable_name = planner_fn.__name__
        module = importlib.import_module(module_name)
        assert hasattr(module, callable_name), (
            f"{planner_name} missing callable '{callable_name}' in module {module_name}"
        )

        module_callable = getattr(module, callable_name)
        assert callable(module_callable), (
            f"{planner_name} callable '{callable_name}' is not callable"
        )
        assert not _callable_is_stubbed(source_text, callable_name), (
            f"{planner_name} callable '{callable_name}' is stubbed"
        )
