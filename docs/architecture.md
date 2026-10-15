# Architecture Overview

`pathplanning` exposes planning through typed Python contracts and the
registry-backed API. Python validates inputs, dispatches planners, adapts data
to native libraries, and converts native results back to public result types.

## Main layers

- Public API and planner metadata live in `pathplanning/api.py` and
  `pathplanning/registry.py`.
- Search, sampling, and kinodynamic planner adapters live under
  `pathplanning/planners/`.
- Canonical spaces, nearest-neighbor indexes, data structures, and geometry
  live in `pathplanning/spaces`, `pathplanning/nn`,
  `pathplanning/data_structures`, and `pathplanning/geometry`.
- Discrete graph search and kinodynamic search run in C++17 native cores.
  Continuous sampling kernels run in the C11 native core. Python owns the API
  contracts and FFI adaptation.
- Plotting and interactive viewers live in `pathplanning/viz` and are loaded
  through explicit visualization calls.

## Canonical references

- Public planner support: [`SUPPORTED_ALGORITHMS.md`](../SUPPORTED_ALGORITHMS.md)
- Native engine details and build instructions: [`native_core.md`](native_core.md)
- Native ABI, ownership, and errors: [`native_abi.md`](native_abi.md)
- Environment and extension architecture: [`environment_architecture.md`](environment_architecture.md)
- Visualization boundaries: [`visualization.md`](visualization.md)
- Benchmark methodology: [`benchmark_contract.md`](benchmark_contract.md)
