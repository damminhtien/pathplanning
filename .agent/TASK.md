# Task: Refactor the Repository Coding-Agent Workflow

## Goal

Make repository guidance compact and route-based, while storing long-running
task requirements, milestones, progress, and decisions in `.agent/` files.

## Constraints

- Preserve the existing public API and runtime code.
- Keep existing user changes outside this documentation task untouched.
- Preserve historical project notes while clearly marking which files are no
  longer the active task state.
- Keep `AGENTS.md` short and focused on invariants, routing, and continuation.

## Acceptance

- `AGENTS.md` routes to focused architecture, invariant, development, and
  benchmark references.
- `.agent/` defines the task lifecycle, fresh-thread prompt, milestone plan,
  current state, and task decision log.
- Old task and handoff files are clearly historical, with stale cross-links
  updated.
- `git diff --check` passes, changed local Markdown links resolve, and
  `graphify update .` succeeds.

## Status

Completed 2026-10-15. Validation evidence is recorded in `.agent/STATE.md`.
