# AI Agent Operating Manual

Last updated: 2026-02-12 (version `0.2.0`)

This document defines how AI agents should operate in `PathPlanning`.

## Objective

Deliver safe, production-quality improvements while preserving deterministic planner behavior, typing guarantees, and import safety.

## Product Directives (Authoritative)

1. Production API scope is registry-driven (`pathplanning.registry`).
2. Keep root imports lightweight and side-effect free.
3. Incomplete or unstable algorithms MUST NOT be exposed through production registry/API.

## Repository Context (Current)

- `pathplanning/planners/search`: search algorithms and facades
- `pathplanning/planners/sampling`: sampling algorithms
- `pathplanning/spaces`: canonical space/environment implementations
- `pathplanning/nn`: nearest-neighbor indexing
- `pathplanning/data_structures`: tree/storage primitives
- `pathplanning/geometry`: geometry/path generation math
- `pathplanning/viz`: plotting-only helpers (lazy imports)

## Required Quality Gates

Run from repo root:

```bash
ruff check .
ruff format --check .
pyright
pytest -q
```

Import-safety smoke checks (recommended on refactors):

```bash
pytest -q tests/test_import_safety.py tests/test_supported_modules_import.py
```

## Working Agreement

1. Keep changes focused; avoid mixing unrelated refactors.
2. Update tests and docs when behavior/API/support scope changes.
3. Preserve deterministic defaults for randomized planners.
4. Do not introduce plotting imports into core runtime modules.
5. Keep package boundaries clear (`spaces/utils/geometry/nn/data_structures`).

## Release Hygiene

For a version bump, update in the same change:

1. `pyproject.toml` (`[project].version`)
2. `README.md` (release metadata and usage notes)
3. `CHANGELOG.md`
4. Agentic files (`state.md`, `tasks.md`, `decisions.md`, `handoff.md`)

## Definition of Done

1. Requested change is complete.
2. Relevant checks are green.
3. Docs and support matrix are synchronized with code.
4. Risks/follow-ups are recorded in `handoff.md`.
