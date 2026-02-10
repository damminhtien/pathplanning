# Contributing

Thanks for helping improve PathPlanning. Please follow these guidelines:

- Keep changes small and focused; separate unrelated fixes.
- Add or update tests for any behavior change.
- Run quality gates locally before pushing:
  - `ruff check .`
  - `pytest -q`
  - `pre-commit run --all-files`
- Update docs (README/SUPPORTED_ALGORITHMS.md) when APIs or support status change.

## Workflow
- Fork or create a feature branch.
- Make changes with clear commit messages (`feat|fix|chore|docs|test|refactor`).
- Submit a pull request with a short summary and validation commands run.

## Code style
- Python 3.9+.
- Keep plotting and planning logic decoupled.
- Use type hints on public interfaces.
- Avoid silent behavior changes; prefer explicit flags or config.
