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
| T-001 | done | critical | Fix all plan2d direct script runtime failures (`NameError` in `main`) | agent | Fixed class-name mismatches in 8 script entrypoints |
| T-002 | done | critical | Add reusable package layout and stable imports | agent | Added `pathplanning` package + dual-mode imports in plan2d/rrt2d |
| T-003 | done | critical | Define supported algorithm matrix and enforce "all complete algorithms" production scope | agent | Added `SUPPORTED_ALGORITHMS.md` + `pathplanning.registry` |
| T-004 | done | critical | Drop incomplete algorithms from production surface | agent | `sampling3d.abit_star` marked dropped and blocked by package loader |
| T-005 | done | high | Add missing runtime dependencies for supported 3D algorithms | agent | Added `pyrr` to `requirements.txt` |
| T-006 | done | high | Add automated smoke tests for all supported algorithms | agent | Added pytest smoke tests for package API + all supported module imports + all search2d planners + representative 2D/3D runtime |
| T-007 | done | high | Update CI to run smoke/integration checks in addition to lint | agent | Workflow now runs `pytest -q` after pre-commit |
| T-008 | todo | medium | Increase lint strictness after runtime failures are resolved | unassigned | Re-enable undefined-name checks (`F821`) incrementally |
| T-009 | todo | medium | Add benchmark script for representative planners | unassigned | Capture runtime + nodes expanded |
| T-010 | done | medium | Document package API usage examples in README | agent | Added package-first examples and production support policy |

## Production Problem Backlog

| Problem ID | Severity | Symptom | Linked Task |
| --- | --- | --- | --- |
| P-001 | critical | plan2d direct scripts crash with `NameError` due class naming mismatches | T-001 |
| P-002 | critical | Package imports fail from repo root because modules use local import assumptions | T-002 |
| P-003 | critical | Incomplete ABIT* implementation has undefined symbols and stubbed methods | T-004 |
| P-004 | high | 3D modules require `pyrr` but dependency is missing in runtime requirements | T-005 |
| P-005 | high | CI passes lint but does not execute algorithm runtime smoke tests | T-006, T-007 |
| P-006 | medium | Conservative lint profile misses undefined-name class errors | T-008 |

## Completed Tasks

| ID | Date | Summary |
| --- | --- | --- |
| C-001 | 2026-02-10 | Added production lint/pre-commit files and CI integration |
| C-002 | 2026-02-10 | Refreshed README and added visual preview gallery |
| C-003 | 2026-02-10 | Added agent operating documents and decision log |
| C-004 | 2026-02-10 | Executed T-001 to T-007 and T-010: package API, support matrix, dependency fix, smoke tests, CI runtime checks |

## Task Intake Template

When adding a task, include:

1. Objective
2. Scope boundaries
3. Acceptance criteria
4. Validation command(s)
5. Risks and rollback notes
