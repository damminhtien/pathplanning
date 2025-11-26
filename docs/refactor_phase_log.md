
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
