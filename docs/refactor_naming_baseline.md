# Refactor Naming/Layout Baseline Snapshot

## Context
This document captures a one-time baseline snapshot before the snake_case naming/layout migration to the target `pathplanning.*` namespace.

Snapshot date: **February 11, 2026**.

## Environment
- Python: `3.13.5`
- ruff: `0.15.0`
- pyright: `1.1.408`
- pytest: `8.4.2`

## Baseline Commands
```bash
pytest -q
ruff check pathplanning/api.py pathplanning/registry.py pathplanning/search2d.py pathplanning/viz tests scripts
pyright --project pyrightconfig.json
python - <<'PY'
import importlib
import pathplanning
from pathplanning.registry import list_supported_algorithms

mods = sorted({spec.module for spec in list_supported_algorithms()})
for module_name in mods:
    importlib.import_module(module_name)
print(f"import_ok: pathplanning + {len(mods)} supported modules")
PY
```

## Results
- `pytest -q`: `9 passed, 2 warnings`
- `ruff check ...`: `All checks passed!`
- `pyright --project pyrightconfig.json`: `0 errors, 0 warnings, 0 informations`
- Import smoke: `import_ok: pathplanning + 39 supported modules`

## Warnings / Risks Observed
These warnings are tracked as baseline debt (not release-blocking failures for this snapshot):

1. Matplotlib `Axes3D` warning during `tests/test_search2d_smoke.py`.
2. `PendingDeprecationWarning` from `numpy.matlib` import at `pathplanning/Sampling_based_Planning/rrt_3D/BIT_star3D.py:20`.

## Rerun Commands
```bash
pytest -q
ruff check pathplanning/api.py pathplanning/registry.py pathplanning/search2d.py pathplanning/viz tests scripts
pyright --project pyrightconfig.json
python - <<'PY'
import importlib
import pathplanning
from pathplanning.registry import list_supported_algorithms

mods = sorted({spec.module for spec in list_supported_algorithms()})
for module_name in mods:
    importlib.import_module(module_name)
print(f"import_ok: pathplanning + {len(mods)} supported modules")
PY
```

## Conclusion for Next Refactor Steps
- Baseline status is **pass** with known warning debt documented above.
- This snapshot is the reference point for upcoming snake_case layout migration and compatibility-shim rollout under `pathplanning.*`.
- No API/interface/type changes are part of this baseline task.
