# Engineering rules

- Target Python `>=3.10`; native sources use C11 or C++17.
- Keep the public planner API registry-driven. Do not expose incomplete or unstable planners.
- Keep root imports lightweight and side-effect free; keep plotting in `pathplanning/viz`.
- Preserve documented API/native-boundary contracts and deterministic planner defaults.
- Behavior or support changes require regression tests and synchronized documentation.
- Do not claim performance gains without representative benchmark evidence.
- Before committing, inspect the diff, run `git diff --check`, and stage only task files.

## Read by topic

- Architecture overview: `docs/architecture.md`; core invariants: `docs/invariants.md`.
- Native implementation and ABI: `docs/native_core.md`, `docs/native_abi.md`.
- Supported planners: `SUPPORTED_ALGORITHMS.md`.
- Coding, build, and test commands: `CODING_CONVENTIONS.md`, `CONTRIBUTING.md`, `Makefile`.
- Benchmark rules: `docs/benchmark_contract.md`.
- Durable project decisions: `decisions.md`.
- Complex or long-running work: `.agent/PLANS.md` and the active files under `.agent/`.

## Continuing work

- Before continuing an active task, read `.agent/TASK.md` and `.agent/STATE.md`; consult `.agent/PLAN.md` and `.agent/DECISIONS.md` for the next unfinished milestone.
- Treat `tasks.md`, `state.md`, and `handoff.md` as historical records, not current task state.
- After relevant source or agent-guidance changes, run `graphify update .`; generated `graphify-out/` stays untracked.
