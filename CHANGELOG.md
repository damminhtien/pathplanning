# Changelog

All notable changes will be documented here.

## Unreleased
- Package renamed to `pathplanning` (metadata).
- Added registry, support matrix, and runtime smoke tests.
- Added publish workflow targets and PyPI-ready metadata.
- Refactored `DynamicRRT3D` with deterministic RNG support, configurable contracts, and pluggable nearest backends.
- Refactored `Environment3D` to canonical snake_case obstacle fields (`aabb`, `aabb_pyrr`, `obb`) with compatibility properties.
- Fixed 3D sampling correctness bugs (`utils_3d.path` mutable default, sampling bias recursion, `rrt_star_3d` timer scope).
- Added tests for headless import safety, determinism, nearest-backend consistency, and env contract coverage.
- Added refactor architecture and migration notes in `docs/rrt3d_refactor.md`.

## 0.1.0
- Initial production-oriented release of search-based and sampling-based planners with visual demos.
