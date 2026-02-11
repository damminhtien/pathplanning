# Refactor Hotspots: Coupling + Magic Numbers

This inventory highlights planner coupling to environment construction, magic-number parameters, direct RNG usage, and plotting side effects. It is a baseline to guide later refactor tasks.

## Hotspot Table

| File | Issue | Proposed Fix (follow-up task) |
| --- | --- | --- |
| `pathplanning/sampling_based_planning/rrt_3d/extend_rrt_3d.py` | Hard-coded `stepsize`, `maxiter`, `goal_prob`; direct `np.random` use; constructs `Environment3D()` internally; calls `visualization()` and `plt.show()` in planner loop. | Introduce config/dataclass for parameters; inject RNG; accept environment in constructor; move plotting behind explicit opt-in or separate visualizer. |
| `pathplanning/sampling_based_planning/rrt_3d/rrt_3d.py` | Hard-coded `stepsize`, `maxiter`; constructs `Environment3D()`; calls `visualization()` and `plt.show()`; uses visualization in core loop. | Add config + injected environment + headless-safe plotting hooks. |
| `pathplanning/sampling_based_planning/rrt_3d/rrt_connect_3d.py` | Hard-coded `stepsize`, `maxiter`; constructs `Environment3D()`; plotting side effects via `visualization()` and `plt.show()`. | Parameterize iteration limits/step size; accept environment; move plotting to optional adapter. |
| `pathplanning/sampling_based_planning/rrt_3d/rrt_star_3d.py` | Hard-coded `maxiter`, `stepsize`, `gamma`, `eta`; constructs `Environment3D()`; calls `visualization()` and `plt.show()`. | Consolidate params into config; inject env; decouple visualization. |
| `pathplanning/sampling_based_planning/rrt_3d/informed_rrt_star_3d.py` | Hard-coded `stepsize`, `gamma`, `eta`, `rgoal`; constructs `Environment3D()`; direct `np.random` usage; plotting inside planner. | Config + RNG injection + env injection; move plotting to adapter. |
| `pathplanning/sampling_based_planning/rrt_3d/bit_star_3d.py` | Hard-coded `maxiter`, radius tweaks (e.g., `self.r = 2`); constructs `Environment3D()`; direct `np.random` usage; plotting inside planner. | Config with explicit rationale; env/RNG injection; move plotting out of core. |
| `pathplanning/sampling_based_planning/rrt_3d/abit_star_3d.py` | Hard-coded `maxiter`, `n`, `lam`; constructs `Environment3D()`; plotting utilities imported at module import time. | Config + env injection; move plotting to optional module. |
| `pathplanning/sampling_based_planning/rrt_3d/fmt_star_3d.py` | Defaults `radius=1`, `n=1000`; constructs `Environment3D()`; direct sampling bias; `plt.show()` and visualization side effects. | Config + env/RNG injection; move plotting to optional visualizer. |
| `pathplanning/sampling_based_planning/rrt_3d/utils_3d.py` | Global functions use `np.random` directly; internal `Environment3D()` construction inside helper class; visualization helper triggers plotting. | Pass RNG explicitly; avoid implicit env construction in helpers; keep visualization strictly optional. |
| `pathplanning/sampling_based_planning/rrt_2d/rrt.py` | Direct `np.random` usage; environment constructed in planner; `iter_max`, `goal_sample_rate`, `step_len` used without config object. | Introduce typed config + RNG injection; environment injection. |
| `pathplanning/sampling_based_planning/rrt_2d/rrt_connect.py` | Direct `np.random` usage; environment constructed in planner; parameters passed via constructor without a config object. | Add config + RNG injection; environment injection. |
| `pathplanning/sampling_based_planning/rrt_2d/rrt_star.py` | Direct `np.random` usage; environment constructed; `search_radius` + `iter_max` used directly; plotting usage in planner output flow. | Config + RNG injection; decouple plotting. |
| `pathplanning/sampling_based_planning/rrt_2d/informed_rrt_star.py` | Direct `np.random` usage; environment constructed; plotting side effects (`plt.show`) within planner. | Config + RNG injection; separate visualization. |
| `pathplanning/sampling_based_planning/rrt_2d/rrt_star_smart.py` | Direct `np.random` and `random` usage; env constructed; magic numbers like `beacons_radius = 2`; plotting side effects. | Config + RNG injection; env injection; extract beacon radius to config. |
| `pathplanning/sampling_based_planning/rrt_2d/extended_rrt.py` | Direct `np.random` usage; env constructed; plotting side effects with `plt.show()`. | Config + RNG injection; env injection; move plotting out. |
| `pathplanning/sampling_based_planning/rrt_2d/dynamic_rrt.py` | Direct `np.random` usage; env constructed; plotting side effects; multiple internal visualization calls. | Config + RNG injection; env injection; separate plotting. |
| `pathplanning/sampling_based_planning/rrt_2d/dubins_rrt_star.py` | Direct `random.random` usage; plotting side effects; magic numbers in demo usage (e.g., radius/search/iter). | Config + RNG injection; isolate plotting; move demo defaults to examples. |
| `pathplanning/sampling_based_planning/rrt_2d/batch_informed_trees.py` | Hard-coded demo constants (e.g., `iter_max = 200`); plotting side effects via `plt.show()`. | Extract config; move demo constants to examples/tests. |
| `pathplanning/sampling_based_planning/rrt_2d/fast_marching_trees.py` | Hard-coded plotting/visualization in planner; env constructed in class. | Config + env injection; optional visualization module. |
| `pathplanning/search_based_planning/plan2d/*` planners | `env.Env()` constructed in planners; plotting side effects (`plt.show()`) inside algorithm; search params tied to env module. | Introduce environment interface + DI; move plotting to separate renderer. |
| `pathplanning/search_based_planning/search_3d/*` planners | `env(resolution=...)` constructed inside planners; plotting via `plot_util_3d.visualization` and `plt.show()` inside algorithms. | Inject environment (or grid) via constructor; decouple visualization from core search. |
| `pathplanning/curves/dubins_path.py`, `reeds_shepp.py` | Step size and curvature parameters used as defaults (magic numbers) and exposed in APIs; direct `plt` usage in helper modules. | Centralize defaults into config/constants; keep plotting in optional helpers. |

## Notes

- Numeric literals are widely embedded as defaults or module-level constants. Prioritize converting planner parameters into typed config objects, and document defaults in one place.
- Direct `np.random` usage should be replaced by injected `np.random.Generator` (already done for `dynamic_rrt_3d.py`) to enable deterministic runs.
- Plotting side effects are currently invoked inside planner algorithms; move to explicit visualization adapters or command-line examples.
