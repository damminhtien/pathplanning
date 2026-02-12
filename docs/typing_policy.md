# Typing Policy

This repository enforces typing incrementally with `pyright` as the primary checker.

## Goals

- Keep public APIs explicitly typed.
- Enforce strict typing in production core modules.
- Allow legacy/demo modules to remain on basic checks during migration.
- Keep type-checking reproducible in local and CI workflows.

## Enforcement Model

- Primary checker: `pyright` (`pyrightconfig.json`).
- Global mode: `basic` for broad coverage.
- Strict scope:
  - `pathplanning/core`
  - `pathplanning/env`
  - `pathplanning/registry.py`
  - `pathplanning/api.py`
  - `pathplanning/__init__.py`

This model gives full-repo signal without blocking progress on legacy code.

## Public API Requirements

- Exported package APIs must have explicit parameter and return annotations.
- `pathplanning/api.py` must avoid unbounded `Any` for return values.
- Type-only regressions are guarded by tests:
  - `tests/test_typing_contracts.py`

## Distribution Requirements (PEP 561)

- The package includes a `py.typed` marker: `pathplanning/py.typed`.
- Packaging metadata includes marker in wheels/sdists:
  - `pyproject.toml` (`[tool.setuptools.package-data]`)
  - `MANIFEST.in`

## Migration Plan

1. Move one legacy module family into strict scope at a time.
2. Keep each migration in small commits with `pytest -q` green.
3. Expand `pyright` strict list only after module family is clean.
