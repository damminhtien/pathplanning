# PathPlanning

PathPlanning is a curated collection of search-based and sampling-based path planning algorithms for robotics, with built-in visualizations for 2D and 3D demos.

The repository is organized for practical use:

- algorithm implementations grouped by planning family
- reusable environment and plotting utilities
- runnable demo scripts
- production-oriented linting and pre-commit checks

## Mission

PathPlanning is a library-first, environment-agnostic core for grid and continuous path-planning workflows. It is designed for reproducible and benchmarkable planner development, with deterministic execution patterns and clear contracts. The stack prioritizes strong typing, actionable diagnostics, and performance-friendly primitives for research-to-production use.

## Release

- Package: `pathplanning`
- Version: `0.1.2`
- Canonical repository: `https://github.com/damminhtien/pathplanning`

## Contents

- [Overview](#overview)
- [Visual Preview](#visual-preview)
- [Repository Layout](#repository-layout)
- [Implemented Algorithms](#implemented-algorithms)
- [Installation](#installation)
- [Package API](#package-api)
- [3D RRT Refactor](#3d-rrt-refactor)
- [Production Support Matrix](#production-support-matrix)
- [Run Demos](#run-demos)
- [Publish to PyPI](#publish-to-pypi)
- [Production Developer Workflow](#production-developer-workflow)
- [Typing Policy](#typing-policy)
- [Code Quality Files](#code-quality-files)
- [CI](#ci)
- [License](#license)

## Overview

This codebase is useful for:

- learning classic planning algorithms
- comparing planners on shared map/obstacle settings
- extending planners while keeping plotting and environment code decoupled

Primary modules:

- `pathplanning/planners/search`: search planners (2D and 3D)
- `pathplanning/planners/sampling`: sampling planners (contract-based and legacy research variants)
- `pathplanning/spaces`: shared environment/space contracts and implementations
- `pathplanning/utils`: reusable data structures and geometry/sampling helpers
- `pathplanning/geometry`: curve generation utilities (Bezier, spline, Dubins, Reeds-Shepp)

## Visual Preview

A small gallery from the built-in animations:

### Search-Based

<p align="center">
  <img src="./assets/gif/search/Astar.gif" alt="A star planning animation" width="360"/>
  <img src="./assets/gif/search/Bi-Astar.gif" alt="Bidirectional A star animation" width="360"/>
</p>
<p align="center">
  <img src="./assets/gif/search/D_star_Lite.gif" alt="D star lite planning animation" width="360"/>
  <img src="./assets/gif/search/ARA_star.gif" alt="ARA star planning animation" width="360"/>
</p>

### Sampling-Based

<p align="center">
  <img src="./assets/gif/sampling/RRT_2D.gif" alt="RRT 2D planning animation" width="360"/>
  <img src="./assets/gif/sampling/RRT_CONNECT_2D.gif" alt="RRT connect 2D planning animation" width="360"/>
</p>
<p align="center">
  <img src="./assets/gif/sampling/FMT.gif" alt="FMT star planning animation" width="360"/>
  <img src="./assets/gif/sampling/BIT2.gif" alt="BIT star planning animation" width="360"/>
</p>

## Repository Layout

```text
.
├── .github/workflows/
│   └── pylint.yml
├── docs/
├── examples/
│   └── worlds/
├── pathplanning/
│   ├── core/
│   ├── geometry/
│   ├── env/
│   ├── planners/
│   │   ├── sampling/
│   │   └── search/
│   ├── spaces/
│   ├── utils/
│   └── viz/
├── scripts/
├── tests/
├── SUPPORTED_ALGORITHMS.md
├── pyproject.toml
├── requirements.txt
├── requirements-dev.txt
├── Makefile
└── README.md
```

## Implemented Algorithms

Search-based (2D/3D):

- Breadth-First Search (BFS)
- Depth-First Search (DFS)
- Best-First Search
- Dijkstra
- A\*
- Bidirectional A\*
- Lifelong Planning A* (LPA*)
- Learning Real-Time A* (LRTA*)
- Real-Time Adaptive A* (RTAA*)
- D\*
- D\* Lite
- Anytime D\*
- Anytime Repairing A* (ARA*)

Sampling-based (2D/3D):

- RRT
- RRT-Connect
- Extended-RRT
- Dynamic-RRT
- RRT\*
- Informed RRT\*
- RRT\* Smart
- FMT\*
- BIT\*
- AIT\*

## Installation

```bash
git clone https://github.com/damminhtien/pathplanning.git
cd pathplanning

python -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
```

Python `>=3.10` is recommended.

## Package API

Import-first usage (production path):

```python
import numpy as np

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
    seed=7,  # deterministic default if omitted
)

print(result.success, result.stop_reason, result.path)
```

Load algorithm modules via registry:

```python
from pathplanning import list_supported_algorithms, load_algorithm_module

for spec in list_supported_algorithms():
    module = load_algorithm_module(spec.algorithm_id)
    print(spec.algorithm_id, module.__name__)
```

## 3D RRT Refactor

The 3D sampling stack includes a production refactor for deterministic execution,
headless-safe imports, and clearer environment contracts.

See:

- `docs/rrt3d_refactor.md`

Highlights:

- contract-based planners: `RrtPlanner` (`rrt.py`) and `RrtStarPlanner` (`rrt_star.py`)
- no matplotlib imports in core planner modules
- injected RNG and parameter dataclass (`RrtParams`) for deterministic runs
- canonical `Environment3D` snake_case obstacle fields: `aabb`, `aabb_pyrr`, `obb`
- explicit angle unit handling for rotations (`rotation_matrix` uses radians)

## Production Support Matrix

Production algorithm support is documented in:

- `SUPPORTED_ALGORITHMS.md`

Policy:

1. Only production-ready algorithms are registered in the package API.
2. The active production scope is intentionally small and contract-driven.
3. Current supported planners: `sampling3d.rrt` and `sampling3d.rrt_star`.

## Publish to PyPI

Build and upload (requires `build` and `twine`, already in `requirements-dev.txt`):

```bash
make build            # creates dist/ artifacts
make publish-test     # upload to TestPyPI (configure ~/.pypirc)
make publish          # upload to PyPI (configure ~/.pypirc)
```

Manual commands:

```bash
python -m build
twine check dist/*
twine upload --repository testpypi dist/*
```

## Run Demos

Run from repository root.

2D search demo (runs multiple algorithms):

```bash
python pathplanning/planners/search/run.py
```

Single 2D search example:

```bash
python pathplanning/planners/search/astar.py
```

2D sampling example:

```bash
python pathplanning/planners/sampling/rrt_grid2d.py
```

3D search example:

```bash
python pathplanning/planners/search/astar_3d.py
```

Generated animations are available under:

- `assets/gif/search`
- `assets/gif/sampling`

## Production Developer Workflow

Install development tooling:

```bash
pip install -r requirements-dev.txt
```

Run lint checks:

```bash
ruff check .
# optional: google-style lint via pylint
make lint-google
```

Run formatter (optional but recommended before commit):

```bash
ruff format .
```

Enable git hooks:

```bash
pre-commit install
```

Run all hooks manually:

```bash
pre-commit run --all-files
```

Run smoke tests:

```bash
pytest -q
```

Shortcut commands:

```bash
make install-dev
make lint
make format
make precommit
make test
```

## Typing Policy

Typing is enforced incrementally with `pyright`.

- Global check mode is `basic` to keep broad repository coverage.
- Core production modules are enforced in `strict` mode:
  - `pathplanning/core`
  - `pathplanning/env`
  - `pathplanning/registry.py`
  - `pathplanning/api.py`
  - `pathplanning/__init__.py`
- Public typing contracts are tested in `tests/test_typing_contracts.py`.
- The package ships `py.typed` for PEP 561 compatibility.

Details:

- `docs/typing_policy.md`

## Code Quality Files

- `pyproject.toml`
  - central `ruff` lint/format configuration
  - conservative default lint rule set focused on correctness issues
- `.pre-commit-config.yaml`
  - syntax/config sanity checks
  - `ruff` lint hook for Python files
  - `pyright` type-check hook for `pathplanning/*`
- `requirements-dev.txt`
  - pinned development dependencies for linting, type-checking, and pre-commit
- `.editorconfig`
  - consistent whitespace/newline/indentation defaults
- `Makefile`
  - common development commands (`install-dev`, `lint`, `format`, `precommit`)

## CI

GitHub Actions workflow: `.github/workflows/pylint.yml`

CI installs `requirements-dev.txt` and runs:

```bash
pre-commit run --all-files --show-diff-on-failure
pytest -q
```

## License

See `LICENSE`.
