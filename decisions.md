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
