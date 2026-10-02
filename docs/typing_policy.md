# Typing Policy

This repository enforces Python typing incrementally with `pyright` as the
primary checker. The exact include and strict lists live in
`pyrightconfig.json`; update this document whenever those lists change.

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
  - `pathplanning/spaces/continuous_3d.py`
  - `pathplanning/spaces/continuous_nd.py`
  - `pathplanning/spaces/grid2d.py`
  - `pathplanning/nn/index.py`
  - `pathplanning/data_structures/tree_array.py`
  - `pathplanning/planners/sampling/rrt.py`
  - `pathplanning/planners/sampling/rrt_star.py`
  - `pathplanning/api.py`
  - `pathplanning/registry.py`
- The broad `include` list also checks supporting adapters, visualization
  helpers, and selected examples in basic mode. Tests are excluded.

The native C/C++ implementation is compiled by `make build-ext`; `pyright`
checks the Python declarations and adapters, not C/C++ types or ABI layout.
Keep the C headers and matching `ctypes.Structure` definitions synchronized as
described in `native_core.md`.

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

1. Move one module family into strict scope at a time.
2. Keep behavior migrations covered by deterministic tests.
3. Expand `pyright` strict list only after the module family is clean.
