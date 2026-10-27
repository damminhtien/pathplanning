# Plan: Test suite audit and reduction

1. [done] Snapshot baseline; collect per-case Python coverage, native LLVM coverage,
   outcomes, planner execution, and uninstrumented timings.
2. [done] Review all test assertions; rank all 357 cases and select <=107 cases with
   critical behavior and 37/37 planner coverage. Preserve independent scenarios.
3. [done] Remove redundant/low-value cases; strengthen retained planner assertions
   where necessary. Measure candidate coverage and accepted coverage losses.
4. [done] Validate clean build, all/unit/slow tests, Ruff, format, Pyright, Graphify,
   ranking integrity, and production-source equality.
5. [done] Inspect final diff and checkpoint the task atomically; Git history
   records the commit containing this validated state.
