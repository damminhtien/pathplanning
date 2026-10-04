# Handoff Notes

Use this file to transfer context between AI agents or from agent to human reviewer.

## Latest Handoff — 2026-10-04

Date: 2026-10-04
Author: AI Agent

### Scope Completed

1. Removed all 30 tracked GIF assets and their packaging configuration; visualization docs use the generated PNG gallery.
2. Removed embedded plotting demos from geometry modules, along with the obsolete geometry drawing and lazy-import helpers. Numeric geometry APIs remain available.
3. Removed legacy planner registry mappings and the callback-based graph-search C ABI. Search ABI is now version 3; custom graph protocols are converted to CSR before native search.
4. Removed stale legacy lint, typing, and packaging configuration.

### Validation Performed

1. Full suite: 154 passed. Ruff checks and formatting passed.
2. Native extensions built successfully; the production search library exports `pp_native_search_plan` and no removed callback entrypoints.
3. Graphify code graph updated successfully; `git diff --check` passed and no GIF files remain in the repository.

### Risks / Follow-ups

Historical changelog and baseline records still describe the layouts and APIs that existed when those records were written.

## Previous Handoff — 2026-10-02

Date: 2026-10-02
Author: AI Agent

### Scope Completed

1. Preserved the existing C++17 discrete search core in `pathplanning/native/search_engine.cpp`.
2. Added a C11 sampling core for RRT, RRT*, Informed RRT*, FMT*, BIT*, ABIT*, RRT-Connect, and DynamicRRT3D tree operations.
3. Kept Python planner modules as API/FFI adapters; built-in continuous spaces use native models, while custom spaces, goals, and supported objectives retain Python callbacks.
4. Built `_search_engine` and `_continuous_engine` as separate extensions with C ABIs. Core migration commit: `d54e7dc`.
5. Updated the architecture, support, build, contribution, state, and handoff documentation. Added `docs/native_core.md` as the detailed reference.

### Validation Performed

1. C11 and C++17 syntax checks with `-Wall -Wextra -Werror` passed.
2. `make build-ext` completed; direct loading found graph-search ABI version `0.4.0` and continuous ABI version `1.0.0`.
3. Ruff, format checks, `compileall`, and `git diff --check` passed.
4. The full pytest suite and native sampling benchmarks were not run. No sampling speedup claim has been established.
5. Graphify updated successfully. Its AST parser reports that it cannot fully extract the two C ABI headers; both compilers accepted the headers and implementations.

### Risks / Follow-ups

1. Custom Python spaces, goals, and supported RRT* objectives can invoke Python callbacks during native planning. Built-in continuous spaces use the callback-free native path.
2. The benchmark script times the complete sampling API path but does not expose a separate sampling-kernel timer or paired pre-migration baseline.
3. The local default `python` is 3.9 although the package requires Python `>=3.10`; direct C ABI loading passed, but package-level import under the supported Python version was not verified in this environment.

## Previous Handoff — 2026-10-01

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
4. `scripts/benchmark_planners.py` now emits the shared v1 benchmark report, retaining every measured and warm-up run with seeds and source/native artifact provenance. Its default five measured repetitions still summarize graph initialization and native search separately.
5. Compared five-run end-to-end medians against the parent commit on the same Python 3.12 host: 2D A* 15.02 ms to 1.27 ms; 3D weighted A* 3.15 ms to 2.34 ms.

### Risks / Follow-ups

1. Custom Python graphs are fully snapshotted before each query and default to a 1,000,000-node limit; reuse `NativeGraph` for repeated queries or larger graphs.
2. The benchmark timings are local and do not include compiler or hardware metadata; use them for same-host comparisons.

## Previous Handoff — 2026-02-12

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
