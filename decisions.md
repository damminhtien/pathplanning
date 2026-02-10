# Engineering Decisions

Track durable decisions so future agents do not re-litigate baseline choices.

## DEC-001: Use `ruff` as baseline linter/formatter

- Date: 2026-02-10
- Status: accepted
- Context: Repository has legacy Python modules; strict lint immediately produces many unrelated failures.
- Decision: Start with conservative correctness-focused `ruff` rules and expand gradually.
- Consequence: Fast adoption now, incremental hardening over time.

## DEC-002: Use `pre-commit` as local and CI lint entrypoint

- Date: 2026-02-10
- Status: accepted
- Context: Developers need one command path for local and CI consistency.
- Decision: Run lint through `.pre-commit-config.yaml` and invoke it in GitHub Actions.
- Consequence: Uniform checks and fewer CI/local mismatches.

## DEC-003: Keep runtime and dev dependencies split

- Date: 2026-02-10
- Status: accepted
- Context: Users running demos should not need dev tooling.
- Decision: Keep runtime deps in `requirements.txt` and dev tooling in `requirements-dev.txt`.
- Consequence: Cleaner onboarding and smaller production runtime environment.

## DEC-004: Production scope covers all supported algorithms

- Date: 2026-02-10
- Status: accepted
- Context: Product requirement is broad algorithm coverage, not a small subset.
- Decision: Maintain production support across all algorithms that pass packaging, runtime smoke checks, and quality gates.
- Consequence: Higher maintenance cost, but complete capability coverage.

## DEC-005: Provide a reusable Python package API

- Date: 2026-02-10
- Status: accepted
- Context: Script-only execution is insufficient for integration use cases.
- Decision: Standardize imports and expose stable package-level APIs for planner execution.
- Consequence: Requires import refactor away from path hacks and stronger API compatibility discipline.

## DEC-006: Drop incomplete algorithms from production surface

- Date: 2026-02-10
- Status: accepted
- Context: Incomplete modules create runtime failures and erode reliability.
- Decision: Mark incomplete algorithms as unsupported and remove them from package exports, CI gates, and production docs until implemented.
- Consequence: Reduced immediate algorithm count in production, increased reliability and clarity.

## DEC-007: Use explicit algorithm registry as production contract

- Date: 2026-02-10
- Status: accepted
- Context: Production scope needs an auditable list of supported vs dropped algorithms.
- Decision: Define algorithm metadata in `pathplanningv2.registry` and publish matrix in `SUPPORTED_ALGORITHMS.md`.
- Consequence: Changes to production support now require explicit registry updates and become easier to review.

## Decision Template

Use this template for future entries:

```text
## DEC-XXX: <title>
- Date: YYYY-MM-DD
- Status: proposed|accepted|deprecated
- Context: ...
- Decision: ...
- Consequence: ...
```
