# AI Agent Operating Manual

This document defines how AI agents should operate in `PathPlanning`.

## Objective

Deliver safe, production-quality improvements to the repository while preserving reproducibility, readability, and developer velocity.

## Product Directives (Authoritative)

1. Production scope must include all supported algorithms in this repository.
2. Delivery must expose a reusable Python package API.
3. Incomplete algorithms are not part of the production surface and must be dropped from support until completed.

## Success Criteria

1. Changes are correct and minimal.
2. Existing demos continue to run.
3. Quality checks pass before handoff.
4. Reusable package API works from repository root imports.
5. Supported algorithm matrix is accurate and enforced.
6. Incomplete algorithms are excluded from production API/docs/CI.
7. Documentation and state are updated for the next agent.

## Repository Context

- `Search_based_Planning/plan2d`: 2D search planners and facade runner
- `Search_based_Planning/Search_3D`: 3D search planners
- `Sampling_based_Planning/rrt_2D`: 2D sampling planners
- `Sampling_based_Planning/rrt_3D`: 3D sampling planners
- `CurvesGenerator`: curve generation modules
- `.github/workflows/pylint.yml`: CI lint workflow (pre-commit based)
- `pyproject.toml`: lint/format settings (`ruff`)
- `.pre-commit-config.yaml`: git hook pipeline

## Working Agreement

1. Make the smallest change that fully solves the request.
2. Prefer backward-compatible edits unless explicitly requested otherwise.
3. Avoid broad refactors mixed with feature work.
4. Do not silently change algorithm behavior.
5. Add or update docs when behavior, commands, or workflow changes.
6. Do not expose experimental/incomplete algorithms through production package exports.

## Required Quality Gates

Run from repository root:

```bash
ruff check .
pre-commit run --all-files
```

Package/import smoke check:

```bash
python -c "import pathplanning; print('import ok')"
```

If code was changed in runnable modules, run at least one relevant smoke demo:

```bash
python Search_based_Planning/plan2d/run.py
python Sampling_based_Planning/rrt_2D/rrt.py
```

## Release Hygiene

For any version bump, update these files in the same change:

1. `pyproject.toml` (`[project].version`)
2. `README.md` (release metadata and install URL if changed)
3. Agentic state files (`state.md`, `tasks.md`, `decisions.md`, `handoff.md`) with current date and validation status

## Standard Workflow

1. Read `state.md`, `tasks.md`, and `decisions.md`.
2. Confirm scope and assumptions.
3. Implement with focused commits.
4. Execute quality gates.
5. Update `state.md` and `tasks.md`.
6. Update supported/dropped algorithm status in docs and decisions.
7. Write a concise handoff in `handoff.md`.

## Coding Standards

1. Keep planning logic separate from plotting/UI concerns.
2. Keep function names explicit and domain meaningful.
3. Prefer deterministic behavior where practical.
4. Add type hints for new/modified public interfaces.
5. Document non-obvious math or algorithm constraints.

## Commit Standards

- Use conventional commit prefixes (`feat`, `fix`, `docs`, `chore`, `refactor`, `test`).
- One logical concern per commit.
- Commit message format:
  - `<type>: <short summary>`

Examples:
- `fix: correct A* neighbor expansion boundary check`
- `docs: add visual preview section to readme`
- `chore: tighten pre-commit lint pipeline`

## Escalation Rules

Escalate before proceeding if any of these happen:

1. Requested change conflicts with current repository decisions in `decisions.md`.
2. A fix requires breaking API/CLI behavior.
3. Validation fails and root cause is outside changed scope.
4. Required input data or expected behavior is ambiguous.

## Definition of Done

A task is done only when:

1. Requested behavior is implemented.
2. `ruff` and `pre-commit` pass.
3. Reusable package imports work in a clean shell.
4. Supported algorithm list matches executable/validated implementations.
5. Docs/state files are updated when relevant.
6. Risks and follow-ups are recorded in `handoff.md`.
