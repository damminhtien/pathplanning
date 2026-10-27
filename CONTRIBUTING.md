# Contributing

Thanks for helping improve PathPlanning.

## Prerequisites

- Python `>=3.10`
- A C11 compiler and a C++17 compiler for source builds
- Install development dependencies:

```bash
pip install -r requirements-dev.txt
make build-ext
```

`pip install .` builds the five native libraries during installation. Users on
the published wheel platforms do not need a native compiler; see the README
installation section for the CPython and OS/architecture matrix. In a checkout,
run `make build-ext` after changing native sources and before invoking Python
planner code directly. `make test-unit`, `make test-slow`, and `make test-all`
build the libraries before running their pytest tier.

## Local Quality Gates

Run these before opening a PR:

```bash
ruff check .
ruff format --check .
pyright
pytest -q
pre-commit run --all-files
```

The retained behavior matrix, test ranking, and accepted coverage losses are
documented in [the test-suite audit](docs/test-suite-audit.md). Prefer strong
behavioral assertions and independent regressions over repeated import/smoke
checks. Each parametrized case counts separately; do not hide independent
scenarios in loops or skips to meet a case budget. The audit's 107-case snapshot
is not a limit on adding necessary regression coverage for future changes.

## Workflow

1. Create a focused branch.
2. Keep changes small and logically grouped.
3. Add/update tests for behavior changes.
4. Update docs (`README.md`, `SUPPORTED_ALGORITHMS.md`) when API or support scope changes.
5. Update `docs/native_core.md` when the C/C++ ABI, native data layout, callback
   boundary, or native build process changes.
6. Submit a PR with:
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
