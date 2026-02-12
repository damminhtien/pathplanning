# Coding Conventions (Google Python Style)

This document defines enforceable repository rules. Keywords use RFC 2119 semantics (MUST/SHOULD/MAY).

## Scope & Tooling

- Code MUST target Python `>=3.10`.
- Line length is 100, LF endings, 4-space indentation.
- `ruff format` output is authoritative.
- `ruff check` and `pyright` MUST pass for touched production modules.

## Naming

- Packages/modules: `lower_snake_case`.
- Classes: `CapWords`.
- Functions/variables/attributes: `lower_snake_case`.
- Constants: `UPPER_SNAKE_CASE`.

## Imports

- Prefer absolute imports (`pathplanning...`) for production code.
- Relative imports MAY be used only within a package when they improve locality and do not cross package boundaries.
- Keep import order: standard library, third-party, first-party.

## Typing

- Public interfaces in `pathplanning/*` MUST be type hinted.
- New/modified core-space-planner API code SHOULD be fully typed.
- Use explicit structured types rather than broad `Any`.

## Linting & Formatting

- Run:

```bash
ruff check .
ruff format --check .
```

- Optional stricter lint for selected files:

```bash
make lint-google
```

## Testing

- Tests MUST be deterministic.
- Randomized tests/planners SHOULD pass seed or RNG explicitly.
- For behavior changes, add tests for both happy path and at least one failure path.
- Baseline test command:

```bash
pytest -q
```

## Architecture Boundaries

- `pathplanning/viz` is plotting-only.
- Core runtime modules (`core`, `spaces`, `nn`, `data_structures`, planner implementations) MUST NOT import `matplotlib` at module import time.
- Shared reusable logic belongs in canonical layers (`spaces`, `utils`, `geometry`, `nn`, `data_structures`), not duplicated per algorithm.

## Error Handling

- Raise specific exceptions (`ValueError`, `TypeError`, `RuntimeError`) with actionable messages.
- Avoid bare `except`.

## Docstrings

- Public functions/classes MUST have clear docstrings.
- Include units and assumptions where relevant.
