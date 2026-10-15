# Repository Invariants

These constraints apply across planner implementations and public interfaces.

## Public API and behavior

- The production planner surface is declared by `pathplanning.registry` and
  summarized in `SUPPORTED_ALGORITHMS.md`. Experimental or incomplete
  planners remain outside that surface until their contracts and tests are
  complete.
- Preserve documented planner parameters, result contracts, and Python/native
  boundary behavior when changing implementations.
- Randomized planners use deterministic defaults; tests pass a fixed seed or
  RNG when randomness affects the expected result.

## Architecture boundaries

- Root package imports stay lightweight and free of runtime side effects.
- Core planning modules do not import plotting libraries at module import
  time. Visualization belongs in `pathplanning/viz` and is explicitly invoked.
- Python owns validation, public contracts, registry dispatch, FFI adaptation,
  and result conversion. Native engines own their search or sampling loops.
- Discrete and kinodynamic native cores use C++17; continuous sampling uses
  C11. Keep ABI declarations, Python layouts, ownership, and ABI versions in
  sync; see `native_abi.md`.
- Python callbacks in native planning remain an explicit compatibility path;
  built-in native models must not acquire implicit callback work in hot loops.

## Quality claims

- Behavior or support changes include regression coverage and updated public
  documentation.
- Performance claims require representative, reproducible benchmark
  evidence. A successful build or focused test alone is not benchmark evidence.
- Source builds target Python `>=3.10`, C11, and C++17.
