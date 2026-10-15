# Task: Diagnose and fix measured performance bottlenecks

## Goal

Audit the current path-planning implementation for unresolved issues, identify
performance bottlenecks with profiling, and fix verified root causes without
expanding the routine test or benchmark workload.

## Constraints

- Use the project Python 3.14 environment and install tools only when needed.
- Keep regression tests small and focused.
- Preserve public APIs, custom callback semantics, and native ABI contracts.
- Run targeted diagnostic benchmarks during investigation; defer broad benchmark
  data reruns until the end.
- Stage explicit task files and commit focused changes after validation.

## Acceptance

- At least one measured bottleneck is fixed at its source and has a regression
  test.
- Focused tests, native build, lint/type checks, and `git diff --check` pass.
- Before/after results use the same deterministic workload and explain limits.
- Graphify is refreshed and all task changes are committed.
