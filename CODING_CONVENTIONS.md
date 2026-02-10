# Coding Conventions (Google Python Style)

This project follows the Google Python Style Guide with pragmatic adjustments for this codebase.

## Essentials
- Python ≥ 3.9.
- Max line length: 100.
- Indent with 4 spaces; LF endings.
- Use absolute imports; avoid relative imports.
- Prefer type hints on public interfaces.
- Document non-trivial functions and classes with Google-style docstrings.
- Keep plotting concerns separate from core planning logic.

## Imports
- Order: standard library, third-party, first-party (`pathplanning`, `Search_based_Planning`, `Sampling_based_Planning`, `CurvesGenerator`).
- Use `from module import name` sparingly; prefer module-qualified access to reduce collisions.
- No wildcard imports.

## Naming
- Modules: `lower_snake_case.py`
- Classes: `CapWords`
- Functions/vars: `lower_snake_case`
- Constants: `UPPER_SNAKE_CASE`

## Docstrings (Google style)
```
def func(arg1: int, arg2: str) -> bool:
    """Summary line.

    Args:
        arg1: Meaning.
        arg2: Meaning.

    Returns:
        What is returned.
    """
```

## Comments
- Keep comments concise and value-adding.
- Use TODO(name): for deferred work.

## Error handling
- Fail fast with clear exceptions; avoid bare `except`.
- Validate inputs at API boundaries.

## Testing
- Add/maintain tests for new behavior (`pytest -q`).
- Run `ruff check .`, `make lint-google` (optional stricter), and `pre-commit run --all-files` before commits.

## Plotting/Demos
- Keep plotting optional and decoupled from algorithm core.
- Use `MPLBACKEND=Agg` in headless/CI contexts.

## Performance
- Avoid premature micro-optimizations; prefer clarity unless hotspots are identified.
