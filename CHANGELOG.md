# Changelog

All notable changes to this project are documented in this file.

## [Unreleased]

### Changed

- Moved discrete search to reusable C++-owned CSR graphs and precomputed goal/heuristic arrays.
- Kept the Python graph protocol through a one-time finite-graph snapshot before search.
- Retained the callback C ABI as a compatibility shim that snapshots before running a native kernel.
- Added `NativeGraph.from_csr` and `NativeGraph.from_edges` for reusable native graph initialization.
- Added direct C++ CSR construction for built-in 2D and 3D grids, using Python only for the valid-node mask.
- Added graph initialization and native search timings to discrete planner stats and benchmarks.
- Benchmark CLI reports medians over configurable repeats (default: 5).
- Removed the unused NetworkX optional extra; CSR arrays are the bulk graph interface.

## [0.2.0] - 2026-02-12

### Changed

- Refactored package layout around reusable layers:
  - planners under `pathplanning/planners/{search,sampling}`
  - canonical spaces under `pathplanning/spaces`
  - geometry utilities under `pathplanning/geometry`
  - NN/tree modules under `pathplanning/nn` and `pathplanning/data_structures`
- Consolidated queue implementations into `pathplanning/utils/priority_queue.py`.
- Shrank root public API to stable entrypoints (`run_planner`, `plan`) and typed aliases.
- Registry now stores explicit planner entrypoints per algorithm.

### Packaging

- Moved demo GIFs out of runtime package paths to `assets/gif/*`.
- Updated packaging config to exclude GIF assets from runtime wheel content.

### Documentation

- Updated README and support matrix for the new module layout and API.

## [0.1.2] - 2026-02-11

### Changed

- Synced package metadata/docs for `0.1.2`.

## [0.1.1] - 2026-02-11

### Changed

- Synced release metadata and restored green lint/test flow in CI.

## [0.1.0] - 2026-02-10

### Added

- Initial production-oriented packaging, registry, and test baseline.
