# Task: Reduce and rank the test suite

Implement the user-approved audit at baseline 8463b69e452fb234ccd054f75774e7b59c79a8ed.
Reduce all maintained pytest cases from 357 toward 71, at most 107. Preserve
behavioral checks for all 37 registry planners and critical correctness,
collision, ABI, stateful, callback, seed and budget regressions. Do not change
production behavior or hide cases with loops, skip/xfail, or alternate tiers.
Deliver docs/test-suite-audit.md and docs/test-suite-ranking.csv, before/after
Python and native coverage, validation, and an atomic commit.
