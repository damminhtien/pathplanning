# Task Decisions: Coding-Agent Workflow

Task-specific decisions for the active workflow refactor. Durable repository
decisions are recorded in `decisions.md`.

## D-1: Separate active task state from project history

- Decision: `.agent/TASK.md`, `.agent/PLAN.md`, `.agent/STATE.md`, and
  `.agent/DECISIONS.md` are the current task record; existing root task/state/
  handoff files remain as history.
- Reason: Old project logs contain useful evidence but should not be mistaken
  for the next unfinished task.

## D-2: Keep task plans verification-centered

- Decision: Milestones carry explicit scope, acceptance criteria, and commands;
  progress advances only after observed validation.
- Reason: Each checkpoint should be independently reviewable and resumable.
