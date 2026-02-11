"""Import and entrypoint contract tests for production-supported algorithm modules."""

from __future__ import annotations

import ast
import importlib
import importlib.util
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


def _entrypoint_is_stubbed(source_text: str, entrypoint_name: str) -> bool:
    tree = ast.parse(source_text)
    for node in tree.body:
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)) and (
            node.name == entrypoint_name
        ):
            return _is_stub_node(node)
    return False


def test_import_all_supported_modules() -> None:
    modules = sorted({spec.module for spec in list_supported_algorithms()})
    for module_name in modules:
        importlib.import_module(module_name)


def test_import_supported_modules_does_not_load_matplotlib() -> None:
    modules = sorted({spec.module for spec in list_supported_algorithms()})
    for module_name in modules:
        code = (
            "import importlib\n"
            "import sys\n"
            f"before = set(sys.modules)\n"
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


def test_import_all_supported_modules_together_does_not_load_matplotlib() -> None:
    """Importing all supported planners in one process must stay headless-safe."""
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
        f"Batch import of supported modules loads matplotlib:\\n{result.stdout}{result.stderr}"
    )


def test_supported_modules_have_non_empty_source_and_expected_entrypoint() -> None:
    """
    Expected entrypoint contract:
    - every supported algorithm module has non-empty source text.
    - every supported algorithm exposes the registry-declared top-level callable/class symbol.
    - entrypoint symbol cannot be a stub (`pass`, `...`, `NotImplementedError` only).
    """
    for algo_spec in list_supported_algorithms():
        source_path = _module_source_path(algo_spec.module)
        source_text = source_path.read_text(encoding="utf-8")
        assert source_text.strip(), (
            f"{algo_spec.algorithm_id} points to an empty module: {source_path}"
        )

        expected_entrypoint = expected_entrypoint_for_algorithm(algo_spec.algorithm_id)
        module = importlib.import_module(algo_spec.module)
        assert hasattr(module, expected_entrypoint), (
            f"{algo_spec.algorithm_id} missing expected entrypoint '{expected_entrypoint}' "
            f"in module {algo_spec.module}"
        )

        entrypoint = getattr(module, expected_entrypoint)
        assert callable(entrypoint), (
            f"{algo_spec.algorithm_id} entrypoint '{expected_entrypoint}' is not callable"
        )
        assert not _entrypoint_is_stubbed(source_text, expected_entrypoint), (
            f"{algo_spec.algorithm_id} entrypoint '{expected_entrypoint}' is stubbed"
        )
