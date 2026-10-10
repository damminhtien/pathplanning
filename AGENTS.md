# Repository Agent Instructions

## Project goals and architecture

- Keep the production API registry-driven (`pathplanning.registry`). Do not expose incomplete or unstable planners through the public API.
- Keep root imports lightweight and side-effect free; keep plotting imports out of core planning modules.
- Preserve deterministic defaults for randomized planners and the documented contracts at Python/native boundaries.
- Search planners live in `pathplanning/planners/search`; sampling planners live in `pathplanning/planners/sampling`; canonical spaces are in `pathplanning/spaces`.
- Nearest-neighbor indexes, data structures, geometry, and visualization helpers live in `pathplanning/nn`, `pathplanning/data_structures`, `pathplanning/geometry`, and `pathplanning/viz`.
- Discrete graph search runs in the C++17 core (`pathplanning/native/search_engine.cpp`). Continuous sampling kernels run in the C11 core (`pathplanning/native/continuous_engine.c`). Python modules own API contracts, registry dispatch, and FFI adaptation.

## Working practices

- Keep changes focused and readable. Update tests and documentation when behavior, API, or support scope changes.
- Follow the contracts and parameter vocabulary of existing planners when adding or changing an algorithm.
- Do not claim performance improvements without representative benchmark evidence.
- Prefer `rtk` for compact exploration and checks; use `rg` and exact file reads when full output matters. Preserve command exit status.
- Before committing, inspect the diff, run `git diff --check`, and stage only files in the requested change.
- Source builds require Python `>=3.10`, a C11 compiler, and a C++17 compiler. Native ABI details are documented in `docs/native_core.md`.

## Validation

Run the relevant checks from the repository root. For native or broad changes, use the full project gates:

```bash
make build-ext
ruff check .
ruff format --check .
pyright
pytest -q
```

For import-safety refactors, also run:

```bash
pytest -q tests/test_import_safety.py tests/test_supported_modules_import.py
```

## Documentation and releases

- Keep `README.md`, `CHANGELOG.md`, and `SUPPORTED_ALGORITHMS.md` synchronized with package behavior and supported scope.
- For a version or release change, synchronize `pyproject.toml`, `README.md`, and `CHANGELOG.md`; review `state.md`, `tasks.md`, `decisions.md`, and `handoff.md` for stale information.
- Record open risks and follow-ups in `handoff.md` when they affect maintainers or users.

## Graphify

- When `graphify-out/graph.json` exists, run `graphify query "<question>"` first for codebase and architecture questions. Use `graphify path "<A>" "<B>"` for relationships and `graphify explain "<concept>"` for focused concepts.
- Use `graphify-out/wiki/index.md` for broad navigation when it exists. Read `GRAPH_REPORT.md` only for broad architecture reviews or when focused queries are insufficient.
- After code changes, run `graphify update .` to refresh the local code graph. `graphify-out/` is generated local state and stays out of Git.
