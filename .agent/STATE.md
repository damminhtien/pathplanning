# State: Test suite audit

Implementation and validation complete; this checkpoint records the audit.
Baseline: 8463b69e452fb234ccd054f75774e7b59c79a8ed (357 cases).
Final: 107 cases, 88 functions, 37 test files; reduction 70.03%.
All 37 canonical registry planners have behavioral witnesses. All 26 selected
P0 cases remain. The CSV ranks every baseline case: 107 keep, 3 merge, 247 drop.
Production sources, API, registry, dependencies and CI remain byte-identical.

Validation: fresh native build; 107 full-suite cases; 105 unit and 2 slow;
pytest 8.4.2 and installed pytest 9.1.1; Ruff, format and Pyright pass.
Ranking integrity, planner execution and production-source equality verified.
Python per-case/subprocess and native LLVM before/after measurements completed.
Uninstrumented local medians: 17.86s baseline, 6.53s candidate (three runs each).
Graphify rebuilt 4398 nodes/12639 edges; four native headers retain extractor
syntax warnings. Generated graph stays ignored; native build/tests pass.

Durable results: docs/test-suite-audit.md and docs/test-suite-ranking.csv.
Raw logs, profiles and source hashes: /tmp/pathplanning-audit-evidence.
Temporary audit harness: /tmp/pathplanning-audit-tools.
Frozen source snapshot: /tmp/pathplanning-suite-audit.
Temporary artifacts are uncommitted and are not permanent reproduction storage.
Supporting-tool and diagnostic coverage losses are explicitly documented.
Unrelated root node_modules/ remains untracked and must be preserved.
No remote CI or push is claimed.
