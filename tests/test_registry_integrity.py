"""Integrity tests for registry-declared planner mappings."""

from __future__ import annotations

import ast
import importlib
import importlib.util
import inspect
from pathlib import Path
import subprocess
import sys

import pytest

from pathplanning.registry import (
    PLANNER_REGISTRY,
    SAMPLING_PLANNERS,
    SEARCH_PLANNERS,
    _build_registry,
    get_continuous_planner,
    get_discrete_planner,
)


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


def _callable_is_stubbed(source_text: str, symbol_name: str) -> bool:
    tree = ast.parse(source_text)
    for node in tree.body:
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)) and (
            node.name == symbol_name
        ):
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


def _iter_registry_entries() -> list[tuple[str, str, object]]:
    return [
        (spec.problem_kind, planner_name, spec.planner)
        for planner_name, spec in PLANNER_REGISTRY.items()
    ]


def test_registered_planner_modules_are_usable() -> None:
    """Every registered planner must expose a concrete, import-safe callable."""
    for kind, planner_name, planner_fn in _iter_registry_entries():
        module_name = planner_fn.__module__
        source_path = _module_source_path(module_name)
        source_text = source_path.read_text(encoding="utf-8")
        assert source_text.strip(), (
            f"{kind}:{planner_name} points to an empty module: {source_path}"
        )
        assert not _imports_matplotlib(source_text), (
            f"{kind}:{planner_name} imports matplotlib in module source: {source_path}"
        )

        module = importlib.import_module(module_name)
        callable_name = planner_fn.__name__
        assert hasattr(module, callable_name), (
            f"{kind}:{planner_name} missing callable '{callable_name}' in module {module_name}"
        )
        module_callable = getattr(module, callable_name)
        assert callable(module_callable), (
            f"{kind}:{planner_name} callable '{callable_name}' is not callable"
        )
        assert not _callable_is_stubbed(source_text, callable_name), (
            f"{kind}:{planner_name} callable '{callable_name}' is stubbed"
        )


def test_importing_all_registered_planner_modules_is_headless() -> None:
    """Importing all registered planner modules must not load matplotlib."""
    modules = sorted({fn.__module__ for _, _, fn in _iter_registry_entries()})
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
        f"Registered planner modules load matplotlib during import:\n{result.stdout}{result.stderr}"
    )


def test_legacy_registry_mappings_derive_from_specs() -> None:
    """Compatibility mappings should be derived from the canonical registry."""
    expected_search = {
        name: spec.planner
        for name, spec in PLANNER_REGISTRY.items()
        if spec.problem_kind == "discrete"
    }
    expected_sampling = {
        name: spec.planner
        for name, spec in PLANNER_REGISTRY.items()
        if spec.problem_kind == "continuous"
    }
    assert dict(SEARCH_PLANNERS) == expected_search
    assert dict(SAMPLING_PLANNERS) == expected_sampling

    for _, _, planner_fn in _iter_registry_entries():
        inspect.signature(planner_fn)


def test_registry_rejects_duplicate_planner_names() -> None:
    """Duplicate IDs must fail at registry construction rather than overwrite."""
    spec = next(spec for spec in PLANNER_REGISTRY.values() if spec.problem_kind == "discrete")
    with pytest.raises(ValueError, match="Duplicate planner name"):
        _build_registry((("duplicate", spec), ("duplicate", spec)))


def test_dispatch_resolves_planners_from_canonical_registry() -> None:
    """Kind-specific dispatch must use the implementation recorded in each spec."""
    for name, spec in PLANNER_REGISTRY.items():
        if spec.problem_kind == "discrete":
            assert get_discrete_planner(name) is spec.planner
            with pytest.raises(KeyError, match="Unknown continuous planner"):
                get_continuous_planner(name)
        else:
            assert get_continuous_planner(name) is spec.planner
            with pytest.raises(KeyError, match="Unknown discrete planner"):
                get_discrete_planner(name)


def test_supported_planner_document_matches_registry() -> None:
    """The checked-in support matrix must be generated from planner metadata."""
    result = subprocess.run(
        [sys.executable, "scripts/generate_supported_algorithms.py", "--check"],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, f"{result.stdout}{result.stderr}"
