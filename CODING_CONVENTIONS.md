# Coding Conventions (Google Python Style)

This document defines enforceable rules for this repository. Keywords use RFC 2119 semantics (MUST/SHOULD/MAY).

## Scope & Tooling

- Code MUST target Python ≥ 3.10 and line length 100 with LF endings and 4-space indents.
- `ruff format` output is authoritative; run via `make format` or `ruff format .`.
- `ruff check --config pyproject.toml` MUST be clean before merge (`make lint` or `pre-commit run --all-files`).
- Google-style lint (`make lint-google`) MAY be used for additional checks.

## Naming

- Modules and packages MUST be `lower_snake_case`.
- Existing CamelCase packages (`Search_based_Planning`, `Sampling_based_Planning`, `CurvesGenerator`) are LEGACY; no new CamelCase modules/packages MAY be added, and new code MUST live under snake_case packages (e.g., `pathplanning`).
- Classes MUST use `CapWords`; functions, variables, and attributes MUST use `lower_snake_case`; constants MUST use `UPPER_SNAKE_CASE`.

## Imports

- Default to absolute imports across packages.
- Relative imports MAY be used only when both conditions hold: (1) staying within the same package tree, and (2) they shorten paths or break intra-package import cycles. Do not use multi-level relative imports (`from ...`) across sibling packages.
- Import order MUST be: standard library, third-party, first-party (`pathplanning`, legacy packages), with ruff/isort enforcing the order. Wildcard imports MUST NOT be used.

## Formatting

- Formatter: `ruff format` MUST be applied to all committed Python files. Configure editors/CI to run it automatically.

## Linting

- Primary linter: `ruff check` with the rule set in `pyproject.toml` (currently correctness-focused E9/F63/F7 plus isort). Lint MUST pass before merging.
- Optional: `pylint` with `.pylintrc` MAY be run for Google-style guidance; failures block only if the reviewer requires.

## Typing

- Type checker: `pyright` (configured in `pyrightconfig.json`).
- Public interfaces in `pathplanning/*` MUST carry type hints; new or modified functions/classes in that package SHOULD be fully typed. Legacy packages are currently exempt but SHOULD add typing when touched.
- Run `pyright` (or `make typecheck`) locally; CI MAY gate on it once the legacy surface is cleaned.

## Error Handling

- Raise specific built-ins: `ValueError`/`TypeError` for bad inputs, `RuntimeError` for unexpected runtime failures, and domain-specific exceptions ONLY if defined in `pathplanning.errors` (add there first).
- Exception messages MUST state the failing variable/value and expected constraints. Avoid bare `except`; catch the narrowest exception type and re-raise with context when necessary.

## Testing

- Tests MUST be deterministic: seed RNGs (e.g., `numpy.random.seed(0)`) in tests that rely on randomness.
- Slow/long-running tests SHOULD be marked with `@pytest.mark.slow`; CI MAY skip them by default.
- New production code MUST include tests covering the happy path and at least one failure path; aim for line/branch coverage growth in `pathplanning` core.
- Command: `pytest -q` (or `make test`).

## Boundaries

- Core (`pathplanning` package) MUST NOT import plotting modules. Plotting/demos MAY import core APIs, not the reverse. Keep algorithm implementations free of UI/plotting side effects.

## Logging

- Core code MUST NOT use `print`. Use the standard `logging` module with module-level loggers. Demos MAY use `print` for quick output, but not in shared library paths.

## Numerical Conventions

- Distances/positions MUST be in meters; angles MUST be in radians unless explicitly documented otherwise.
- Use right-handed coordinate frames for 3D; document deviations inline.
- Default floating tolerance (epsilon) SHOULD be `1e-9` for geometric comparisons unless algorithm papers require a different value; document chosen epsilons.
- Randomized planners SHOULD expose a seed parameter and pass it through to RNGs used internally.

## Docstrings & Comments

- Public functions/classes MUST have Google-style docstrings describing arguments, return values, side effects, and units where applicable.
- TODOs MUST follow `TODO(name): detail` and SHOULD include a link to a task/issue when available.

## Enforcement Gaps (TODO)

- Wire `pyright` into CI and pre-commit after legacy modules are incrementally typed.
