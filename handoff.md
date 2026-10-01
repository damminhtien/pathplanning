# Handoff Notes

Use this file to transfer context between AI agents or from agent to human reviewer.

## Latest Handoff

Date: 2026-10-01
Author: AI Agent

### Scope Completed

1. Moved discrete search to C++-owned CSR graphs with dense native search state and precomputed goal/heuristic arrays.
2. Added `NativeGraph.from_csr` and `NativeGraph.from_edges`, including SciPy CSR array interoperability without a SciPy runtime dependency.
3. Added direct C++ CSR construction for built-in 2D/3D grids and retained the Python graph protocol through a bounded one-time snapshot.
4. Preserved the callback C ABI as a graph-snapshot compatibility shim and corrected bidirectional search to use reverse edges.
5. Added end-to-end and phase timings to discrete search results and the benchmark output; removed the unused NetworkX extra.

### Files Changed

1. Native graph storage and FFI: `pathplanning/native/{graph.py,_ffi.py,search_engine.cpp,search_engine.h}`.
2. Search adapter/contracts and grid fast paths: `pathplanning/planners/search/_internal/native.py`, `pathplanning/core/contracts.py`, `pathplanning/spaces/grid{2d,3d}.py`.
3. API docs, changelog, benchmark, dependency metadata, and `tests/test_native_graph_search.py`.

### Validation Performed

1. Built the native extension with Python 3.12 and C++17.
2. `python -m pytest -q`: 93 passed.
3. `ruff check .`, `ruff format --check .`, `pyright`, and `git diff --check`: passed.
4. `scripts/benchmark_planners.py --json` reports medians over five runs by default and separates graph initialization from native search.
5. Compared five-run end-to-end medians against the parent commit on the same Python 3.12 host: 2D A* 15.02 ms to 1.27 ms; 3D weighted A* 3.15 ms to 2.34 ms.

### Risks / Follow-ups

1. Custom Python graphs are fully snapshotted before each query and default to a 1,000,000-node limit; reuse `NativeGraph` for repeated queries or larger graphs.
2. The benchmark timings are local and do not include compiler or hardware metadata; use them for same-host comparisons.

## Previous Handoff

Date: 2026-02-12
Author: AI Agent

### Scope Completed

1. Reviewed and refreshed root non-code files to reflect the current architecture and release state.
2. Updated release/docs metadata for version `0.2.0` consistency.
3. Rewrote stale agentic files (`agent.md`, `state.md`, `tasks.md`, `decisions.md`) with current module layout and priorities.
4. Updated contribution/style guidance (`CONTRIBUTING.md`, `CODING_CONVENTIONS.md`) for Python `>=3.10` and current quality gates.
5. Refreshed changelog entries to include `0.2.0` highlights.

### Validation Performed

1. `pytest -q tests/test_packaging_py_typed.py` (previously green during version bump).
2. Additional full checks should be run if config files (`pyrightconfig.json`, `.pre-commit-config.yaml`) change further.

### Open Items

1. Align CI workflow paths in `.github/workflows/pylint.yml` with current package layout.
2. Expand registry-backed production support beyond current two sampling planners.
3. Establish benchmark baselines for key planners.

## Handoff Template

Copy this section for the next handoff:

### Scope Completed

1. ...

### Files Changed

1. ...

### Validation Performed

1. ...

### Risks / Follow-ups

1. ...
