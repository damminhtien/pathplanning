# Project State

Last updated: 2026-02-11
Branch: `refactor/naming`

## Current Snapshot

- Project: `PathPlanning`
- Package version: `0.1.2`
- Canonical repository: `https://github.com/damminhtien/pathplanning`
- Language: Python
- Runtime deps: `requirements.txt`
- Dev deps: `requirements-dev.txt`
- Linting: `ruff` via `pyproject.toml`
- Hooks: `.pre-commit-config.yaml`
- CI lint entry: `.github/workflows/pylint.yml`
- Product directives:
  - Production scope includes all supported algorithms
  - Deliver reusable Python package API
  - Drop incomplete algorithms from production surface

## Recent Completed Work

1. README refreshed with current repository layout and developer workflow.
2. Added production-oriented dev tooling:
   - `pyproject.toml`
   - `.pre-commit-config.yaml`
   - `.editorconfig`
   - `Makefile`
   - `requirements-dev.txt`
3. CI lint flow switched from direct pylint command to pre-commit pipeline.
4. README visual preview gallery added using existing GIF assets.
5. Added agent operating documents (`agent.md`, `state.md`, `tasks.md`, `handoff.md`, `decisions.md`).
6. Fixed plan2d script runtime failures caused by wrong class names in `main()`.
7. Added reusable package API under `pathplanning`.
8. Added supported algorithm registry and support matrix (`SUPPORTED_ALGORITHMS.md`).
9. Dropped incomplete `sampling3d.abit_star` from production API surface.
10. Added runtime dependency `pyrr` for 3D modules.
11. Added pytest smoke tests and CI runtime test execution.
12. Fixed 3D package-mode/runtime issues:
   - local queue import resolution in 3D search modules
   - numpy-safe emptiness check in 3D plot line drawing utilities
13. Standardized naming and Google-style docstrings for 3D RRT modules.
14. Fixed local/CI pre-commit formatting drift and restored green CI lint pipeline.
15. Updated package metadata and documentation for `0.1.1` patch release.

## Current Production Blockers

1. Lint profile remains conservative; undefined-name checks are not fully enforced yet.
2. Benchmark/regression baseline for planner performance is still missing.
3. `Sampling_based_Planning/rrt_3D/ABIT_star3D.py` remains in repository as incomplete source (dropped from production support, not yet removed/rewritten).

## Active Priorities

1. Increase lint strictness incrementally (`F821` and related correctness checks).
2. Add benchmark script for representative planners.
3. Decide future strategy for incomplete ABIT* source file (remove vs complete implementation).

## Environment Notes

Recommended Python: `>=3.10`

Useful commands:

```bash
pip install -r requirements-dev.txt
make lint
make precommit
```

## Validation Snapshot

Recent observed behavior:

1. `python Search_based_Planning/plan2d/run.py` succeeds with all listed planners.
2. Individual previously failing plan2d scripts now execute without `NameError`.
3. Package import works from repo root: `import pathplanning`.
4. Representative 3D script run works headless (`MPLBACKEND=Agg python Search_based_Planning/Search_3D/Astar3D.py`).
5. Smoke tests pass: `pytest -q`.
6. CI lint workflow passes for latest branch commit (`pre-commit` green on push + pull_request events).

## Update Protocol

When completing work, append a short entry with:

1. Date
2. Change summary
3. Validation commands run
4. Open risks / next step
