# PathPlanningV2

PathPlanningV2 is a curated collection of search-based and sampling-based path planning algorithms for robotics, with built-in visualizations for 2D and 3D demos.

The repository is organized for practical use:
- algorithm implementations grouped by planning family
- reusable environment and plotting utilities
- runnable demo scripts
- production-oriented linting and pre-commit checks

## Contents

- [Overview](#overview)
- [Repository Layout](#repository-layout)
- [Implemented Algorithms](#implemented-algorithms)
- [Installation](#installation)
- [Run Demos](#run-demos)
- [Production Developer Workflow](#production-developer-workflow)
- [Code Quality Files](#code-quality-files)
- [CI](#ci)
- [License](#license)

## Overview

This codebase is useful for:
- learning classic planning algorithms
- comparing planners on shared map/obstacle settings
- extending planners while keeping plotting and environment code decoupled

Primary modules:
- `Search_based_Planning/plan2d`: 2D grid search planners
- `Search_based_Planning/Search_3D`: 3D search planners
- `Sampling_based_Planning/rrt_2D`: 2D sampling-based planners
- `Sampling_based_Planning/rrt_3D`: 3D sampling-based planners
- `CurvesGenerator`: curve generation utilities (Bezier, spline, Dubins, Reeds-Shepp)

## Repository Layout

```text
.
├── .github/workflows/
│   └── pylint.yml
├── CurvesGenerator/
├── Sampling_based_Planning/
│   ├── gif/
│   ├── rrt_2D/
│   └── rrt_3D/
├── Search_based_Planning/
│   ├── gif/
│   ├── plan2d/
│   └── Search_3D/
├── .pre-commit-config.yaml
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
- A*
- Bidirectional A*
- Lifelong Planning A* (LPA*)
- Learning Real-Time A* (LRTA*)
- Real-Time Adaptive A* (RTAA*)
- D*
- D* Lite
- Anytime D*
- Anytime Repairing A* (ARA*)

Sampling-based (2D/3D):
- RRT
- RRT-Connect
- Extended-RRT
- Dynamic-RRT
- RRT*
- Informed RRT*
- RRT* Smart
- FMT*
- BIT*
- ABIT*
- AIT*

## Installation

```bash
git clone https://github.com/damminhtien/PathPlanningV2.git
cd PathPlanningV2

python -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
```

Python `>=3.9` is recommended.

## Run Demos

Run from repository root.

2D search demo (runs multiple algorithms):

```bash
python Search_based_Planning/plan2d/run.py
```

Single 2D search example:

```bash
python Search_based_Planning/plan2d/astar.py
```

2D sampling example:

```bash
python Sampling_based_Planning/rrt_2D/rrt.py
```

3D search example:

```bash
python Search_based_Planning/Search_3D/Astar3D.py
```

Generated animations are available under:
- `Search_based_Planning/gif`
- `Sampling_based_Planning/gif`

## Production Developer Workflow

Install development tooling:

```bash
pip install -r requirements-dev.txt
```

Run lint checks:

```bash
ruff check .
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

Shortcut commands:

```bash
make install-dev
make lint
make format
make precommit
```

## Code Quality Files

- `pyproject.toml`
  - central `ruff` lint/format configuration
  - conservative default lint rule set focused on correctness issues
- `.pre-commit-config.yaml`
  - syntax/config sanity checks
  - `ruff` lint hook for Python files
- `requirements-dev.txt`
  - pinned development dependencies for linting and pre-commit
- `.editorconfig`
  - consistent whitespace/newline/indentation defaults
- `Makefile`
  - common development commands (`install-dev`, `lint`, `format`, `precommit`)

## CI

GitHub Actions workflow: `.github/workflows/pylint.yml`

CI installs `requirements-dev.txt` and runs:

```bash
pre-commit run --all-files --show-diff-on-failure
```

## License

See `LICENSE`.
