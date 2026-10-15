# Plan: Refactor the Coding-Agent Workflow

| Milestone | Verification boundary | Status |
| --- | --- | --- |
| M1. Define document ownership | Root instructions route to stable architecture, invariants, development, and benchmark references. | done |
| M2. Add long-running task workflow | `.agent/` explains task setup, verification-boundary milestones, checkpoints, fresh threads, and compaction recovery. | done |
| M3. Migrate existing guidance | Historical root records and old cross-links no longer claim to be active task state. | done |
| M4. Validate and checkpoint | Review scoped diff, resolve changed local links, pass `git diff --check`, refresh Graphify, then commit only this task. | done |
