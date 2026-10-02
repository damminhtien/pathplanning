# Project State

Last updated: 2026-10-02
Branch: `main`

## Current Snapshot

- Project: `PathPlanning`
- Package: `pathplanning` (`0.2.0`)
- Canonical repository: `https://github.com/damminhtien/pathplanning`
- Python requirement: `>=3.10`
- Source build toolchain: C11 and C++17 compilers
- Runtime dependencies: `requirements.txt`
- Development dependencies: `requirements-dev.txt`

## Production API Surface

The registry-backed production planners are enumerated in
[`SUPPORTED_ALGORITHMS.md`](SUPPORTED_ALGORITHMS.md). The current matrix covers
discrete BFS, DFS, greedy best-first, A*, Dijkstra, weighted A*, anytime A*,
bidirectional Dijkstra, and bidirectional A*, plus continuous RRT, RRT*,
Informed RRT*, FMT*, BIT*, ABIT*, and RRT-Connect.

`DynamicRRT3D` is a separate stateful public API, not a registry entry.

## Architecture Snapshot

- Discrete graph search, CSR storage, and frontier queues run in the C++17 core
  at `pathplanning/native/search_engine.cpp`.
- Continuous sampling planner loops and DynamicRRT3D pruning/growth run in the
  C11 core at `pathplanning/native/continuous_engine.c`.
- Python owns contracts, registry dispatch, input/result adaptation, and the
  compatibility callbacks for custom Python spaces, goals, and objectives.
- Built-in continuous spaces pass bounds and obstacle arrays to C once per
  plan; their sampling and collision checks run natively.
- Native architecture, data ownership, build requirements, and limitations are
  detailed in [`docs/native_core.md`](docs/native_core.md).

## Latest Native-Core Change

Core migration commit: `d54e7dc` (`Move continuous planner cores to native C`).
The existing C++ search engine was retained. The C and C++ engines build as
separate libraries and export C ABIs for the thin Python adapters.

## Validation and Evidence

For the native-core migration, C/C++ syntax checks with warnings as errors,
extension builds, direct C ABI loading, Ruff, Python compilation, and
`git diff --check` passed. The full pytest suite and a sampling performance
benchmark were not run as part of that migration; do not claim measured
end-to-end speedups until benchmark evidence is recorded.

## Active Risks and Gaps

1. Custom Python spaces, goal predicates, and supported RRT* objectives still
   invoke Python callbacks during native planner operations. Built-in space
   models use callback-free native sampling and collision checks.
2. The benchmark currently has representative discrete and sampling rows, but
   only discrete rows expose separate graph-init/native-search timings.
3. Graphify's AST parser does not fully extract the two native C ABI headers;
   C and C++ compilers accept the headers and source files.
