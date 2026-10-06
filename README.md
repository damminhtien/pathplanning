# PathPlanning

PathPlanning is a Python package for search-based and sampling-based path-planning algorithms,
with reusable planner contracts and optional visualization utilities.

## Release

- Package: `pathplanning`
- Version: `0.2.0`
- Canonical repository: `https://github.com/damminhtien/pathplanning`

## Overview

Current codebase organization:

- `pathplanning/native`: C++ graph-search and C sampling-planner cores exposed through a stable C ABI
- `pathplanning/planners/search`: Python planner wrappers around the native search core
- `pathplanning/planners/sampling`: thin Python interfaces for native sampling planners
- `pathplanning/spaces`: canonical environment/configuration-space layer
- `pathplanning/nn`: nearest-neighbor index abstractions
- `pathplanning/data_structures`: reusable tree/storage structures
- `pathplanning/utils`: shared utilities (including priority queue)
- `pathplanning/geometry`: geometry and trajectory generation utilities
- `pathplanning/viz`: optional 2D/3D rendering and offline Matplotlib viewer

## Repository Layout

```text
.
├── pathplanning/
│   ├── core/
│   ├── native/       # C++ graph search and C sampling engines
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

Production API support is intentionally small and planner-registry driven.

See `SUPPORTED_ALGORITHMS.md` for the canonical matrix.

The continuous registry provides `rrt`, `rrt_star`, `informed_rrt_star`,
`fmt_star`, `bit_star`, `abit_star`, and `rrt_connect`. Stateful `DynamicRRT3D`
also runs tree pruning and growth in the native C engine. The discrete registry
includes native `bidirectional_dijkstra` and `bidirectional_astar`; the other
registered discrete searches also run through the C++ graph-search core.

Sampling planners execute their search loops, trees, queues, geometry checks,
and nearest-neighbor queries in C. Python validates inputs, converts built-in
spaces or an explicit `NativeContinuousSpaceModel` to native data, and adapts
path results. Python callbacks are disabled by default; pass
`RrtParams(allow_python_callbacks=True)` to use a custom space, goal predicate,
or RRT* objective that has no native representation.

See [Native Planning Core](docs/native_core.md) for the architecture, callback
boundary, build requirements, and benchmark scope. The [native ABI contract](docs/native_abi.md)
defines ABI versions, ownership, callback errors, and incompatible-library
handling.

For a replayable diagnostic run, pass `trace=TraceOptions()` and then call
`view_result(scene_from_problem(problem), result)`. Normal planning uses a
production library compiled without trace code. See the
[visualization guide](docs/visualization.md) for controls, custom scenes, and
the plotting API migration notes.

### Native model for a custom space

A custom space can avoid Python callbacks by returning native bounds and
supported obstacle arrays from `to_native_model()`:

```python
from pathplanning.native import NativeContinuousSpaceModel


class UnitLineSpace:
    def to_native_model(self) -> NativeContinuousSpaceModel:
        return NativeContinuousSpaceModel(lower_bounds=[0.0], upper_bounds=[1.0])
```

The space's operations must have the same uniform sampling, Euclidean distance,
steering, and collision semantics as the returned model. Otherwise, enable the
Python compatibility path explicitly with `RrtParams(allow_python_callbacks=True)`.

## MovingAI Scenario Gallery

These scenes use real map and start/goal pairs from the
[Moving AI 2D benchmark collection](https://movingai.com/benchmarks/grids.html).
Each image shows one matrix, the MovingAI scenario's endpoints, an algorithm's
route, and its explored/frontier cells. The examples cover maze, room, DAO,
street, and StarCraft map families under the `land_octile_v1` movement rules.

The gallery chooses a median path-length A* scenario from each family's saved
work-pass cohort, then replays the named planner with a bounded diagnostic trace
to draw its search state. Those trace runs create these illustrations only; the
benchmark's latency figures come from separate production runs with tracing and
metrics instrumentation disabled. See the
[scenario and benchmark reproduction steps](docs/shortest_path_benchmark_reproduction.md).

<p align="center">
  <img src="./assets/images/movingai-maze-scenario.png" alt="A-star path and explored frontier on the real MovingAI maze512-1-0 map and scenario" width="49%"/>
  <img src="./assets/images/movingai-room-scenario.png" alt="Bidirectional A-star path and two search fronts on a MovingAI room map scenario" width="49%"/>
</p>
<p align="center">
  <img src="./assets/images/movingai-dao-scenario.png" alt="Weighted A-star path on a MovingAI Dragon Age Origins grid map scenario" width="49%"/>
  <img src="./assets/images/movingai-street-scenario.png" alt="A-star path and explored cells on a MovingAI Denver street grid scenario" width="49%"/>
</p>
<p align="center">
  <img src="./assets/images/movingai-sc1-scenario.png" alt="A-star path and explored cells on a MovingAI StarCraft grid map scenario" width="640"/>
</p>

For interactive traces, custom scenes, and the rendering API, see the
[visualization guide](docs/visualization.md).

## Installation

```bash
git clone https://github.com/damminhtien/pathplanning.git
cd pathplanning

python -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install .
```

Python `>=3.10`, a C11 compiler, and a C++17 compiler are required when building
from source. Graph search runs in C++; continuous planner kernels run in C.

## Package API

Minimal import-first usage:

```python
from pathplanning import RrtParams, plan_continuous
from pathplanning.core.contracts import ContinuousProblem, GoalState
from pathplanning.spaces.continuous_3d import ContinuousSpace3D

space = ContinuousSpace3D(lower_bound=[0, 0, 0], upper_bound=[10, 10, 10])
params = RrtParams(max_iters=2_000, step_size=0.6, goal_sample_rate=0.1)
problem = ContinuousProblem(
    space=space,
    start=[1.0, 1.0, 1.0],
    goal=GoalState(state=[9.0, 9.0, 1.0], radius=0.5, distance_fn=space.distance),
)

result = plan_continuous(
    problem,
    planner="rrt",
    params=params,
    seed=7,
)

print(result.success, result.stop_reason, result.path)
```

Single public API for both planner families:

```python
from pathplanning.api import plan_discrete
from pathplanning.core.contracts import DiscreteProblem
from pathplanning.spaces.grid2d import Grid2DSearchSpace

problem = DiscreteProblem(
    graph=Grid2DSearchSpace(),
    start=(5, 5),
    goal=(45, 25),
)
result = plan_discrete(problem, planner="astar", seed=0)
print(result.success, result.iters)
```

### Native graph input

Discrete searches run against a C++-owned CSR graph. For repeated queries or
large graphs, initialize it once and reuse it:

```python
from pathplanning.api import plan_discrete
from pathplanning.core.contracts import DiscreteProblem
from pathplanning.native import NativeGraph

graph = NativeGraph.from_edges(
    nodes=[0, 1, 2, 3],
    edges=[(0, 1, 1.0), (0, 2, 4.0), (1, 3, 2.0), (2, 3, 1.0)],
)
problem = DiscreteProblem(graph=graph, start=0, goal=3)
result = plan_discrete(problem, planner="astar")
```

For an exact point goal, the informed sampling planners can be selected through
the same API:

```python
from pathplanning.api import plan_continuous
from pathplanning.core.contracts import ContinuousProblem, GoalState
from pathplanning.spaces.continuous_3d import ContinuousSpace3D

space = ContinuousSpace3D(lower_bound=[0, 0, 0], upper_bound=[10, 10, 10])
problem = ContinuousProblem(
    space=space,
    start=[1.0, 1.0, 1.0],
    goal=GoalState(state=[9.0, 9.0, 1.0]),
    params={"sample_count": 1_000, "batch_size": 100},
)
result = plan_continuous(problem, planner="bit_star", seed=7)
```

`fmt_star`, `bit_star`, `abit_star`, and `informed_rrt_star` optimize additive
path length and reject custom objectives. `bidirectional_astar` requires an
exact goal and a consistent graph heuristic; its native kernel validates the
heuristic on the prepared graph before searching.

For bulk input, pass CSR row offsets, neighbor IDs, and edge costs to
`NativeGraph.from_csr`. This also accepts arrays from a SciPy CSR matrix through
its `indptr`, `indices`, and `data` attributes; SciPy is not required by the
native graph API. No NetworkX conversion is used.

Existing Python graphs that implement `neighbors` and `edge_cost` remain
supported. Their finite reachable graph is copied into CSR once before each
search. Goal predicates and heuristics are evaluated during that preparation;
the C++ search loop does not call Python. The default materialization limit is
1,000,000 nodes and can be changed with
`DiscreteProblem(params={"max_materialized_nodes": ...})`. Use `NativeGraph`
for graphs that exceed that limit or for repeated searches over the same graph.
The built-in 2D and 3D grids pass their valid-node mask and motion set to C++,
which builds their CSR adjacency without Python edge callbacks.

Discrete search results include `graph_init_s` and `native_search_s` stats, and
`elapsed_s` covers graph preparation, native search, and path adaptation. The
two canonical benchmark commands emit the versioned
`pathplanning_benchmark_v1` report with raw per-run observations, summaries,
seed/workload settings, source and native artifact fingerprints, and host
metadata. Use `--output path/to/report.json` to save it atomically or `--json`
to print it. Reports saved under `benchmark-results/` are excluded from source
fingerprints. See [the benchmark contract](docs/benchmark_contract.md) for the
schema and interpretation limits.

## Run Demos

Run from repository root.

```bash
python examples/worlds/custom_grid_world.py
python examples/worlds/demo_3d_world.py
python examples/viewer_2d.py
python examples/viewer_3d.py
python scripts/benchmark_planners.py --output benchmark-results/planners.json
python scripts/benchmark_native_sampling.py --output benchmark-results/native-sampling.json
```

## Developer Workflow

Install dev dependencies:

```bash
pip install -r requirements-dev.txt
make build-ext
```

Run checks:

```bash
ruff check .
ruff format --check .
pyright
make test       # fast unit tier (excludes slow integration/packaging checks)
make test-slow  # slow integration/packaging tier
make test-all   # full suite
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
