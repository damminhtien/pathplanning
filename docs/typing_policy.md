# Typing Policy

This repository enforces typing incrementally with `pyright` as the primary checker.
`pyright` is the authoritative type checker for merge decisions.

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
  - `pathplanning/sampling_based_planning`
  - `pathplanning/registry.py`
  - `pathplanning/api.py`
  - `pathplanning/__init__.py`
- Basic scope:
  - `pathplanning/viz`
  - `examples`
  - `pathplanning/_legacy`
  - wrapper/demo-heavy modules while they are being migrated

This model gives full-repo signal without blocking progress on legacy code.

## Public API Requirements

- Exported package APIs must have explicit parameter and return annotations.
- `pathplanning/api.py` must avoid unbounded `Any` for return values.
- Type-only regressions are guarded by tests:
  - `tests/test_typing_contracts.py`

## Typing Rules

- No implicit `Optional` in public APIs.
- Prefer `Protocol` over concrete inheritance when defining planner contracts.
- Avoid `Any`; if unavoidable, isolate and document the boundary.
- Prefer `numpy.typing` (`NDArray`, typed aliases) for array-facing APIs.

## Distribution Requirements (PEP 561)

- The package includes a `py.typed` marker: `pathplanning/py.typed`.
- Packaging metadata includes marker in wheels/sdists:
  - `pyproject.toml` (`[tool.setuptools.package-data]`)
  - `MANIFEST.in`

## Migration Plan

1. Move one legacy module family into strict scope at a time.
2. Keep each migration in small commits with `pytest -q` green.
3. Expand `pyright` strict list only after module family is clean.
