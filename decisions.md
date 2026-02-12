# Engineering Decisions

Track durable decisions so future agents do not re-litigate baseline choices.

## DEC-001: Use `ruff` as baseline linter/formatter

- Date: 2026-02-10
- Status: accepted
- Decision: Use `ruff` as the primary lint/format tool, with pragmatic gradual tightening.

## DEC-002: Use `pre-commit` as local + CI lint entrypoint

- Date: 2026-02-10
- Status: accepted
- Decision: Keep checks runnable through a single pre-commit pipeline for local consistency.

## DEC-003: Keep runtime and development dependencies split

- Date: 2026-02-10
- Status: accepted
- Decision: Runtime deps in `requirements.txt`; dev tooling in `requirements-dev.txt`.

## DEC-004: Keep a registry-backed production API surface

- Date: 2026-02-10
- Status: accepted
- Decision: Production-supported algorithms are explicitly declared in `pathplanning.registry`.
- Consequence: Algorithm exposure changes require explicit metadata + tests.

## DEC-005: Keep root package imports lightweight

- Date: 2026-02-10
- Status: accepted
- Decision: `pathplanning/__init__.py` must avoid heavy runtime imports and side effects.

## DEC-006: Separate runtime math/logic from visualization

- Date: 2026-02-12
- Status: accepted
- Decision: Plotting helpers stay in `pathplanning/viz`; runtime modules must not import matplotlib at module import time.

## DEC-007: Use canonical reusable layers across planners

- Date: 2026-02-12
- Status: accepted
- Decision: Shared components belong in dedicated layers (`spaces`, `nn`, `data_structures`, `utils`, `geometry`) rather than per-algorithm copies.

## DEC-008: Exclude heavy GIF assets from runtime package

- Date: 2026-02-12
- Status: accepted
- Decision: GIF assets live under `assets/gif/*` and are excluded from runtime wheel payload.

## DEC-009: Keep release metadata synchronized

- Date: 2026-02-12
- Status: accepted
- Decision: Any version bump updates `pyproject.toml`, `README.md`, `CHANGELOG.md`, and agentic state files in the same change.

## Update Log

- 2026-02-11: Synced release metadata and agentic files for version `0.1.2`.
- 2026-02-12: Completed `0.2.0` metadata/docs sync and architecture doc refresh.

## Decision Template

```text
## DEC-XXX: <title>
- Date: YYYY-MM-DD
- Status: proposed|accepted|deprecated
- Context: ...
- Decision: ...
- Consequence: ...
```
