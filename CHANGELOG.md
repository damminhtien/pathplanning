# Changelog

All notable changes to this project are documented in this file.

## [Unreleased]

- No unreleased entries yet.

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
