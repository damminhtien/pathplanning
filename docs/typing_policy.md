# Typing policy

`pyright` is the authoritative type checker for this repository.

## Modes

- **Strict zones**: `pathplanning/core`, `pathplanning/env`, `pathplanning/sampling_based_planning/rrt_3d`.
- **Basic zones**: legacy planners, visualizers, tests, and examples.

This split keeps production-facing APIs strongly typed while allowing incremental migration for legacy modules.

## Rules

1. Public APIs must have explicit annotations.
2. New code in strict zones should avoid `Any`.
3. Use shared aliases from `pathplanning.core.types` for numerical arrays and node identifiers.
4. Do not add plotting imports to core planning modules.

## Validation

Run locally from the repository root:

```bash
pyright
```
