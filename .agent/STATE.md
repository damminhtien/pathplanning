# Current Task State

Status: complete

## Findings

- Project interpreter: `.venv/bin/python`, Python 3.14.8. Scalene 2.3.0 is
  installed in this environment for in-process profiling.
- macOS `sample` could not inspect the process without elevated access and
  `xcrun xctrace` is unavailable. cProfile, Scalene, native counters, and fixed
  workload timings were usable.
- BARN `world_188`, seed 2245405222, 347 circles, 512 samples: PRM* native build
  was 253–260 ms before the change and 7.49 ms in the final run (8.00 ms median
  full API over five runs). Native motion checks fell from 14,372 to 7,554,
  equal to the candidate count. The same no-path result remained.
- On the same `world_188`, RIT* with default CARM went from 3.306 s to 0.650 s
  median in the final three-run profile. It still returns the same seeded path
  cost 10.847130442173992. With CARM disabled, the same workload took about 48 ms;
  this isolated the Gaussian collision field as the dominant cost.
- A successful PRM* case, BARN `world_236`, retained the same path cost
  10.003912169734035 across five runs; final post-change API median was 6.26 ms.
- Scalene attributed 66% of the profiled workload time to the native planner
  call. The C static analyzer reported the same five warnings on the original
  `HEAD` source and the working source; none were introduced by this change.

## Progress

- Native continuous motions now use exact segment checks for box, sphere, and
  OBB models. Native PRM* validates each static undirected edge once; custom
  callback roadmaps still validate both directions.
- RIT* reuses per-plan workspace arrays. The 2D CARM path indexes collision
  feedback spatially and omits Gaussian tails with a contribution bound of
  1e-14; other dimensions retain the full sum.
- Added focused sparse-collision and PRM check-count/direction regressions.
- Final checks passed on Python 3.14.8: native extension build; 21 focused
  planner and import-safety tests; repository-wide Ruff lint; touched-file
  formatting; Pyright with the project interpreter; and `git diff --check`.
- `ruff format --check .` still reports nine unrelated files that are not
  formatted. The touched Python files pass the formatter check.
- Graphify was refreshed after the native changes. The static analyzer reports
  the same five warnings on the base revision and working source, with no new
  warnings introduced.

## Completion

The focused performance fixes, regression tests, documentation, and final
validation are complete and recorded in the task commit.
