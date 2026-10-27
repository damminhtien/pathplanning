# Decisions: Test suite audit

- User chose whole-repository reduction counted after pytest parameterization.
- Target 71 cases; up to 107 is allowed for independent important behavior.
- Every registry planner must have behavioral assertions, not import-only tests.
- Critical behavior takes precedence over preserving aggregate line coverage.
- Baseline source is frozen in /tmp/pathplanning-suite-audit; audit output and
  temporary instrumentation stay outside Git. Production sources stay unchanged.
- Retain 107 cases, the approved upper allowance, for additional independent
  stateful, callback, budget and collision contracts. This is not a proof that
  71 is impossible or that 107 is minimal.
- Accept measured supporting-tool and diagnostic coverage losses; do not infer
  preserved fault detection from line coverage retention.
- Normalize the pre-existing DynamicRRT sampling fixture with seed 4 and zero
  goal/waypoint probabilities on both measurement sides to remove entropy flakiness.
