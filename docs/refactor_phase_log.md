
## 2026-10-02 - Phase: Native C/C++ planner core and documentation sync

### Architecture Outcome
- Preserved the discrete graph-search engine in `pathplanning/native/search_engine.cpp` (C++17).
- Added the C11 sampling engine in `pathplanning/native/continuous_engine.c` for registry planners and DynamicRRT3D tree operations.
- Kept Python planner modules as typed API/FFI adapters. Built-in continuous spaces use native bounds and obstacle arrays; custom Python spaces, goals, and supported objectives retain callbacks.
- Added `docs/native_core.md`; synchronized README, support matrix, build and contribution guidance, current state, task board, and handoff.

### Validation Record
- C11/C++17 syntax checks with `-Wall -Wextra -Werror`: passed.
- Native extension build and direct C ABI loading: passed.
- Ruff, Python compilation, and `git diff --check`: passed.
- The full pytest suite and sampling benchmark were not run in this phase.
- Graphify updated. Its AST parser still cannot fully extract `continuous_engine.h` and `search_engine.h`; the compilers accepted both headers.

### Open Evidence
- Benchmark built-in native spaces and callback-backed custom spaces end to end, with compiler and host metadata.
- Run the planner regression suite under a supported Python version before making a release claim.

## 2026-02-12T17:55:56+07:00 - Phase: Baseline + Architecture Guards

### Baseline Runs
- `python -m pytest -q`
  - Result: `76 passed in 39.19s`
- `python -m pyright`
  - Result: `1 error`
  - Error:
    - `pathplanning/spaces/continuous_3d.py:179:9`
    - `ContinuousSpace3D.steer` parameter name mismatch with `ContinuousSpace` contract (`step` vs `step_size`).

### Legacy Hotspots (from `rg`)
Command:
- `rg -n "ConfigurationSpace|BatchConfigurationSpace|rrt_grid2d|search2d|_legacy2d_common|plot_util_3d|astar_3d|dstar_lite_3d|anytime_dstar_3d|utils_3d" README.md docs pathplanning tests scripts examples`

Hits:
- `scripts/benchmark_planners.py` (uses `pathplanning.search2d`)
- `tests/test_search2d_smoke.py` (uses `pathplanning.search2d`)
- `tests/test_utils_3d_sample_free.py` + `docs/rrt3d_refactor.md` (legacy `utils_3d` wording)
- `docs/environment_architecture.md` (mentions `search2d` and `rrt_grid2d`)
- `docs/refactor_baseline.md` + `docs/refactor_baseline_run.md` + `docs/refactor_hotspots.md` (historical legacy references)
- `tests/test_architecture_invariants.py` (legacy-symbol negative assertions)

### Guarding Strategy for This Phase
- Add `tests/test_architecture_guards.py` with:
  - hard fail: no `matplotlib` imports inside `pathplanning/planners/*`
  - hard fail: removed legacy modules stay absent (`rrt_grid2d`, `_legacy2d_common`, `plot_util_3d`)
  - hard fail: legacy core symbols absent (`ConfigurationSpace`, `BatchConfigurationSpace`)
  - soft fail (`xfail`): `pathplanning.search2d` pending final removal; convert to hard fail in final cleanup phase.

### Post-Guard Validation
- `python -m pytest -q`
  - Result: `81 passed, 1 xfailed in 40.42s`
  - Note: `xfailed` is the temporary soft guard for `pathplanning.search2d` removal.
- `python -m pyright`
  - Result: `0 errors, 0 warnings, 0 informations`
