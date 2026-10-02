# Task Board

Use this as a lightweight execution board for AI agents.

## Status Legend

- `todo`
- `in_progress`
- `blocked`
- `done`

## Active Tasks

| ID | Status | Priority | Task | Owner | Notes |
| --- | --- | --- | --- | --- | --- |
| T-101 | done | high | Align CI checks with current package layout | agent | `.github/workflows/pylint.yml` checks the current package, tests, scripts, and examples. |
| T-102 | done | high | Expand registry-backed production planners | agent | Current supported entries are listed in `SUPPORTED_ALGORITHMS.md`. |
| T-103 | done | high | Benchmark native sampling planners end to end | agent | Five-seed full-API and C-kernel comparison for all seven planners; see `docs/benchmarks/native_sampling_2026-10-02.md` and JSON. |
| T-104 | done | medium | Synchronize project documentation with the native C/C++ architecture | agent | README, support matrix, build guidance, state, handoff, and architecture docs refreshed on 2026-10-02. |
| T-105 | done | high | Make the continuous native path callback-free by default | agent | Built-in or declared `NativeContinuousSpaceModel` uses native operations; custom-space callbacks require explicit opt-in in registry planners and `DynamicRRT3D`. |
| T-106 | done | medium | Repair Graphify extraction of C ABI headers | local | Ignored `.codex/workflows` patch routes guarded C ABI headers through Graphify's C++ parser after removing linkage guards; both headers now extract without parse warnings. |

## Recent Completed Work

| ID    | Date       | Summary |
| ----- | ---------- | ------- |
| C-101 | 2026-02-12 | Migrated curve modules to `pathplanning/geometry` and moved plotting helper to `pathplanning/viz/geometry_draw.py` |
| C-102 | 2026-02-12 | Updated import smoke tests to `pathplanning.geometry` |
| C-103 | 2026-02-12 | Moved GIF assets to `assets/gif/*` and excluded GIFs from runtime packaging |
| C-104 | 2026-02-12 | Refined package API/registry (`run_planner`, entrypoint-aware registry contracts) |
| C-105 | 2026-02-12 | Bumped package version to `0.2.0` and synced release metadata |
| C-106 | 2026-10-02 | Moved continuous planner expansion and DynamicRRT3D tree operations into native C while retaining the C++ graph-search core |
| C-107 | 2026-10-02 | Updated current docs and labeled earlier refactor reports as historical snapshots |
| C-108 | 2026-10-02 | Added callback-free custom continuous space models, explicit Python callback opt-in, and native sampling benchmarks |

## Intake Template

For each new task, include:

1. Objective
2. Scope boundaries
3. Acceptance criteria
4. Validation commands
5. Risks / rollback notes
