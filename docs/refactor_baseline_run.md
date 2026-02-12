## Baseline Run - 2026-02-12

### 1) `python -m pytest -q`

```text
...................................................
................      [100%]
67 passed in 50.77s
```

### 2) `python -m pyright`

```text
0 errors, 0 warnings, 0 informations
```

### 3) Top 10 duplicated utility concepts (hotspots)

1. 3D Euler rotation matrix helper (`rotation_matrix` / `R_matrix`)
   Files: `pathplanning/sampling_based_planning/rrt_3d/env_3d.py`, `pathplanning/search_based_planning/search_3d/env_3d.py`
2. Default 3D obstacle presets (`get_blocks`/`getblocks`, `get_balls`/`getballs`)
   Files: `pathplanning/sampling_based_planning/rrt_3d/env_3d.py`, `pathplanning/search_based_planning/search_3d/env_3d.py`
3. AABB builders from block bounds (`get_aabb_pyrr`+`get_aabb_list` vs `getAABB`+`getAABB2`)
   Files: `pathplanning/sampling_based_planning/rrt_3d/env_3d.py`, `pathplanning/search_based_planning/search_3d/env_3d.py`
4. 3D box primitive models (AABB/OBB center-extents-orientation containers)
   Files: `pathplanning/sampling_based_planning/rrt_3d/env_3d.py`, `pathplanning/search_based_planning/search_3d/env_3d.py`
5. Dynamic obstacle/world mutators (`new_block`/`New_block`, `move_block`, `move_obb`/`move_OBB`)
   Files: `pathplanning/sampling_based_planning/rrt_3d/env_3d.py`, `pathplanning/search_based_planning/search_3d/env_3d.py`
6. FIFO/LIFO queue wrappers (`QueueFIFO`, `QueueLIFO`)
   Files: `pathplanning/sampling_based_planning/rrt_2d/queue.py`, `pathplanning/search_based_planning/plan2d/utils/queue.py`, `pathplanning/search_based_planning/search_3d/queue.py`
7. Simple heap priority queue wrapper (`QueuePrior`)
   Files: `pathplanning/sampling_based_planning/rrt_2d/queue.py`, `pathplanning/search_based_planning/plan2d/utils/queue.py`, `pathplanning/search_based_planning/search_3d/queue.py`
8. Removable min-heap priority queue concept (`MinHeapPriorityQueue` / `MinheapPQ`)
   Files: `pathplanning/sampling_based_planning/rrt_3d/queue.py`, `pathplanning/search_based_planning/search_3d/queue.py`
9. 2D plotting workflow (grid + obstacle draw + visited/path animation)
   Files: `pathplanning/sampling_based_planning/rrt_2d/plotting.py`, `pathplanning/search_based_planning/plan2d/utils/plotting.py`
10. 3D geometry/collision utility kernel (ray, distance, inside checks, line-vs-shape checks, collide gate)
   Files: `pathplanning/sampling_based_planning/rrt_3d/utils_3d.py`, `pathplanning/search_based_planning/search_3d/utils_3d.py`, `pathplanning/sampling_based_planning/rrt_2d/utils.py`

