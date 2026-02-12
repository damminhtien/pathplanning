# Contributing

Thanks for helping improve PathPlanning.

## Prerequisites

- Python `>=3.10`
- Install development dependencies:

```bash
pip install -r requirements-dev.txt
```

## Local Quality Gates

Run these before opening a PR:

```bash
ruff check .
ruff format --check .
pyright
pytest -q
pre-commit run --all-files
```

## Workflow

1. Create a focused branch.
2. Keep changes small and logically grouped.
3. Add/update tests for behavior changes.
4. Update docs (`README.md`, `SUPPORTED_ALGORITHMS.md`) when API or support scope changes.
5. Submit a PR with:
   - short summary
   - risk notes (if any)
   - validation commands and outputs

## Commit Guidelines

- Use conventional prefixes: `feat`, `fix`, `refactor`, `test`, `docs`, `chore`.
- Prefer one logical concern per commit.

## Design Guidelines

- Keep core planning logic independent from visualization code.
- Prefer absolute imports in production modules.
- Preserve deterministic behavior for randomized planners (explicit seed/RNG flow).
- Keep public interfaces typed.
