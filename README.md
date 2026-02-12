# PathPlanning

PathPlanning is a Python package for search-based and sampling-based path-planning algorithms,
with reusable planner contracts and optional visualization utilities.

## Release

- Package: `pathplanning`
- Version: `0.2.0`
- Canonical repository: `https://github.com/damminhtien/pathplanning`

## Overview

Current codebase organization:

- `pathplanning/planners/search`: search planners (2D/3D modules)
- `pathplanning/planners/sampling`: sampling planners (2D/3D modules)
- `pathplanning/spaces`: canonical environment/configuration-space layer
- `pathplanning/nn`: nearest-neighbor index abstractions
- `pathplanning/data_structures`: reusable tree/storage structures
- `pathplanning/utils`: shared utilities (including priority queue)
- `pathplanning/geometry`: geometry and trajectory generation utilities
- `pathplanning/viz`: plotting-only helpers with lazy imports

## Repository Layout

```text
.
├── pathplanning/
│   ├── core/
│   ├── planners/
│   │   ├── search/
│   │   └── sampling/
│   ├── spaces/
│   ├── nn/
│   ├── data_structures/
│   ├── utils/
│   ├── geometry/
│   └── viz/
├── tests/
├── docs/
├── assets/
├── scripts/
├── pyproject.toml
└── README.md
```

## Production Support Matrix

Production API support is intentionally small and registry-driven.

- `sampling3d.rrt` -> `pathplanning.planners.sampling.rrt:RrtPlanner`
- `sampling3d.rrt_star` -> `pathplanning.planners.sampling.rrt_star:RrtStarPlanner`

See `SUPPORTED_ALGORITHMS.md` for the canonical matrix.

## Visual Preview

Animations are stored at:

- `assets/gif/search`
- `assets/gif/sampling`

Example gallery:

<p align="center">
  <img src="./assets/gif/search/Astar.gif" alt="A* planning animation" width="360"/>
  <img src="./assets/gif/search/Bi-Astar.gif" alt="Bidirectional A* planning animation" width="360"/>
</p>
<p align="center">
  <img src="./assets/gif/sampling/RRT_2D.gif" alt="RRT 2D planning animation" width="360"/>
  <img src="./assets/gif/sampling/RRT_CONNECT_2D.gif" alt="RRT Connect 2D planning animation" width="360"/>
</p>

## Installation

```bash
git clone https://github.com/damminhtien/pathplanning.git
cd pathplanning

python -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
```

Python `>=3.10` is required.

## Package API

Minimal import-first usage:

```python
from pathplanning import RrtParams, run_planner
from pathplanning.spaces.continuous_3d import ContinuousSpace3D

space = ContinuousSpace3D(lower_bound=[0, 0, 0], upper_bound=[10, 10, 10])
params = RrtParams(max_iters=2_000, step_size=0.6, goal_sample_rate=0.1)

result = run_planner(
    "sampling3d.rrt",
    space=space,
    start=[1.0, 1.0, 1.0],
    goal=([9.0, 9.0, 1.0], 0.5),
    params=params,
    seed=7,
)

print(result.success, result.stop_reason, result.path)
```

Registry-driven module loading:

```python
from pathplanning import list_supported_algorithms, load_algorithm_module

for spec in list_supported_algorithms():
    module = load_algorithm_module(spec.algorithm_id)
    print(spec.algorithm_id, module.__name__)
```

## Run Demos

Run from repository root.

```bash
python pathplanning/planners/search/run.py
python pathplanning/planners/search/astar.py
python pathplanning/planners/sampling/rrt_grid2d.py
python pathplanning/planners/search/astar_3d.py
```

## Developer Workflow

Install dev dependencies:

```bash
pip install -r requirements-dev.txt
```

Run checks:

```bash
ruff check .
ruff format --check .
pyright
pytest -q
```

Convenience commands:

```bash
make install-dev
make lint
make typecheck
make test
```

## Typing Policy

Typing is enforced incrementally with `pyright`.

- Global mode: `basic`
- Strict subset:
  - `pathplanning/core`
  - `pathplanning/spaces/continuous_3d.py`
  - `pathplanning/spaces/continuous_nd.py`
  - `pathplanning/spaces/grid2d.py`
  - `pathplanning/nn/index.py`
  - `pathplanning/data_structures/tree_array.py`
  - `pathplanning/planners/sampling/rrt.py`
  - `pathplanning/planners/sampling/rrt_star.py`
  - `pathplanning/api.py`
  - `pathplanning/registry.py`

The package ships `pathplanning/py.typed` (PEP 561).

Details: `docs/typing_policy.md`

## CI

GitHub Actions workflow: `.github/workflows/pylint.yml`

Current CI jobs run:

- `ruff check ...` and `ruff format --check ...`
- `pyright`
- `pytest -q`

## License

See `LICENSE`.
