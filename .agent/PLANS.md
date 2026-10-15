# Long-Running Task Workflow

Use the files in this directory as the current task record for work that spans
multiple files, needs several verification steps, or may continue in a fresh
thread. Keep one active task per checkout; use separate worktrees for
independent concurrent tasks.

## File roles

- `TASK.md` freezes the goal, constraints, scope, and acceptance criteria.
- `PLAN.md` divides the work into independently verifiable milestones.
- `STATE.md` records the latest verified progress and the exact next action.
- `DECISIONS.md` records decisions made for the active task. Promote durable
  repository decisions to `decisions.md` when closing the task.

## Start and execute

1. Check `STATE.md` before broad exploration. If a task is active, continue its
   next unfinished milestone. If it is complete, use the new user request as
   the source of the next task.
2. For a new multi-file or long-running task, write `TASK.md`, `PLAN.md`,
   `STATE.md`, and `DECISIONS.md` before implementation. Keep small, obvious
   changes out of this workflow.
3. Split the plan by verification boundary. Each milestone lists its scope,
   acceptance criteria, and exact validation commands.
4. Complete one milestone as `implement -> validate -> checkpoint -> update
   state`. Repair failed validation before moving to the next milestone.
   Create a focused commit when the verified milestone is a useful checkpoint;
   never include unrelated work.
5. Update `STATE.md` after observing results. Record what passed, what failed,
   current implementation state, known issues, and the exact next step. Do not
   report intended or partial work as complete.
6. When the task finishes, mark it complete, record final validation, and
   promote lasting decisions to `decisions.md`. A later substantial task
   replaces the active task files; Git history retains prior versions.

## Fresh thread and compaction

Use this prompt when continuing in a new thread:

```text
Read AGENTS.md, .agent/TASK.md, and .agent/STATE.md.
Continue the current task from .agent/STATE.md; consult .agent/PLAN.md and
.agent/DECISIONS.md for the next unfinished item. Inspect only additional files
required for that item.
```

After the first compaction, continue if close to a milestone. After two or more
compactions, checkpoint verified work, update `STATE.md`, and start a fresh
thread from the prompt above. If repeated compaction is followed by rereading
the same files without progress, checkpoint and reset context sooner.
