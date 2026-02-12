# Task Board

Use this as a lightweight execution board for AI agents.

## Status Legend

- `todo`
- `in_progress`
- `blocked`
- `done`

## Active Tasks

| ID    | Status      | Priority | Task                                                                 | Owner | Notes |
| ----- | ----------- | -------- | -------------------------------------------------------------------- | ----- | ----- |
| T-101 | todo        | high     | Align CI workflow paths/check targets with the current package layout | agent | `.github/workflows/pylint.yml` still contains legacy path targets |
| T-102 | todo        | high     | Expand production registry to additional stable planners              | agent | Must include import safety + deterministic test coverage |
| T-103 | todo        | medium   | Add benchmark baselines for search and sampling planners              | agent | Extend `scripts/benchmark_planners.py` and publish baseline numbers |
| T-104 | in_progress | medium   | Keep root non-code docs synchronized with current architecture        | agent | README/agentic docs/config refresh work |

## Recent Completed Work

| ID    | Date       | Summary |
| ----- | ---------- | ------- |
| C-101 | 2026-02-12 | Migrated curve modules to `pathplanning/geometry` and moved plotting helper to `pathplanning/viz/geometry_draw.py` |
| C-102 | 2026-02-12 | Updated import smoke tests to `pathplanning.geometry` |
| C-103 | 2026-02-12 | Moved GIF assets to `assets/gif/*` and excluded GIFs from runtime packaging |
| C-104 | 2026-02-12 | Refined package API/registry (`run_planner`, entrypoint-aware registry contracts) |
| C-105 | 2026-02-12 | Bumped package version to `0.2.0` and synced release metadata |

## Intake Template

For each new task, include:

1. Objective
2. Scope boundaries
3. Acceptance criteria
4. Validation commands
5. Risks / rollback notes
