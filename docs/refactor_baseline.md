# Refactor Baseline Report

## Commands Run

- `python -m pytest -q`
  Summary: 19 passed in 8.68s
- `python -m ruff check .`
  Summary: All checks passed!
- `python -m pyright`
  Summary: 0 errors, 0 warnings, 0 informations
- `python -c "import pathplanning; print('import ok')"`
  Summary: import ok

## Failures

- Tests: none
- Imports: none

## sys.path Hacks

- None found via `rg -n "sys\.path|append\(.*sys\.path|insert\(.*sys\.path" pathplanning`.

## Matplotlib Imports At Module Import Time

The following modules call `lazy_import("matplotlib...")` at module import time:

- `pathplanning/curves/bezier_path.py`
- `pathplanning/curves/bspline_curve.py`
- `pathplanning/curves/draw.py`
- `pathplanning/curves/dubins_path.py`
- `pathplanning/curves/quintic_polynomial.py`
- `pathplanning/curves/reeds_shepp.py`
- `pathplanning/sampling_based_planning/rrt_2d/batch_informed_trees.py`
- `pathplanning/sampling_based_planning/rrt_2d/dubins_rrt_star.py`
- `pathplanning/sampling_based_planning/rrt_2d/dynamic_rrt.py`
- `pathplanning/sampling_based_planning/rrt_2d/extended_rrt.py`
- `pathplanning/sampling_based_planning/rrt_2d/fast_marching_trees.py`
- `pathplanning/sampling_based_planning/rrt_2d/informed_rrt_star.py`
- `pathplanning/sampling_based_planning/rrt_2d/plotting.py`
- `pathplanning/sampling_based_planning/rrt_2d/rrt_connect.py`
- `pathplanning/sampling_based_planning/rrt_2d/rrt_star_smart.py`
- `pathplanning/sampling_based_planning/rrt_3d/abit_star_3d.py`
- `pathplanning/sampling_based_planning/rrt_3d/bit_star_3d.py`
- `pathplanning/sampling_based_planning/rrt_3d/extend_rrt_3d.py`
- `pathplanning/sampling_based_planning/rrt_3d/fmt_star_3d.py`
- `pathplanning/sampling_based_planning/rrt_3d/informed_rrt_star_3d.py`
- `pathplanning/sampling_based_planning/rrt_3d/plot_util_3d.py`
- `pathplanning/sampling_based_planning/rrt_3d/rrt_3d.py`
- `pathplanning/sampling_based_planning/rrt_3d/rrt_connect_3d.py`
- `pathplanning/sampling_based_planning/rrt_3d/rrt_star_3d.py`
- `pathplanning/search_based_planning/plan2d/anytime_dstar.py`
- `pathplanning/search_based_planning/plan2d/dstar.py`
- `pathplanning/search_based_planning/plan2d/dstar_lite.py`
- `pathplanning/search_based_planning/plan2d/lifelong_planning_astar.py`
- `pathplanning/search_based_planning/plan2d/utils/plotting.py`
- `pathplanning/search_based_planning/search_3d/anytime_dstar_3d.py`
- `pathplanning/search_based_planning/search_3d/astar_3d.py`
- `pathplanning/search_based_planning/search_3d/bidirectional_astar_3d.py`
- `pathplanning/search_based_planning/search_3d/dstar_3d.py`
- `pathplanning/search_based_planning/search_3d/dstar_lite_3d.py`
- `pathplanning/search_based_planning/search_3d/lp_astar_3d.py`
- `pathplanning/search_based_planning/search_3d/lrt_astar_3d.py`
- `pathplanning/search_based_planning/search_3d/plot_util_3d.py`
- `pathplanning/search_based_planning/search_3d/rta_astar_3d.py`
