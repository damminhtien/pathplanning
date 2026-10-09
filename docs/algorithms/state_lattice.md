# State Lattice

State Lattice runs native A* over discretized SE(2) poses connected by a
robot's supplied local-frame motion primitives. The space owns the primitive
set; every edge is footprint-checked along its sampled trajectory. The search
reports pose samples and each sample's forward/reverse direction.

The bundled helpers provide bicycle-model Ackermann moves and differential
drive moves, including in-place rotations. Custom spaces can provide their own
`StateLatticePrimitive` values. A primitive contains relative `(x, y, yaw)`
samples, one direction (`1` or `-1`), and a positive cost. Samples must follow
the physical motion at a spacing small enough to represent its curvature. The
cost must not underestimate the sampled translation and rotation measured by
the space's `rotation_radius`.

`StateLatticeParams` controls position and heading discretization, collision
sampling, goal tolerance, search budgets, and optional weighted A*. With weight
1, A* is optimal within the finite lattice induced by the provided primitives
and their costs. A weight above 1 favors faster first solutions and gives up
that optimality claim. The primitive set and discretization limit reachability;
this does not claim a resolution-completeness guarantee for arbitrary user sets.
Collision checks interpolate between primitive samples at `collision_step` and
test the full rectangular footprint against occupied cells.

Built-in occupancy grids run entirely in C++. A custom `StateLatticeSpace`
validity callback requires the explicit `StateLatticeParams(allow_python_callbacks=True)`
opt-in. The example shows both reference motion models.

Reference: M. Pivtoraiko and A. Kelly, “Efficient Constrained Path Planning
via Search in State Lattices,” i-SAIRAS 2005,
[paper](https://www.cs.cmu.edu/~alonzo/pubs/papers/isairas05Planning.pdf).

Runnable example: [`examples/state_lattice.py`](../../examples/state_lattice.py).
Run from the repository root with `python -m examples.state_lattice`.
