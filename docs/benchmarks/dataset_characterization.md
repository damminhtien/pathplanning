# Characterizing benchmark datasets before comparing planners

Research and tooling snapshot: 2026-10-08. This work characterizes inputs; it
does not rank planners or report new planner performance results.

The population is the current single-agent MovingAI 2D collection (including
terrain) and corrected 3D collection, DIMACS Challenge 9 core roads and Rome99,
the 300 static BARN worlds, all four Monash voxel collections, four pinned OMPL
geometric demo definitions and all OMPL.app problem configurations. Older 2D
duplicates, MAPF, dynamic navigation, restricted DIMACS contributions and
kinodynamic execution are outside the measured population. OMPL control configurations are retained and identified
so that they cannot accidentally enter a geometric comparison.

## Three levels of classification

Dataset names identify provenance, not graph structure or universal difficulty.
Keep these levels separate:

1. **Problem semantics:** explicit graph versus implicit grid versus continuous
   configuration space; direction, state dimension, motion rules, robot geometry,
   cost units/objective, goal policy and constraints. These determine which
   planners and correctness references are applicable.
2. **Map or graph structure:** size, degree distribution, edge costs, components,
   obstacle density, distance to obstacles and spatial arrangement. These
   explain mechanisms affecting search, without using a candidate's runtime.
3. **Query distribution:** endpoint placement, displacement, direction, reference
   reachability, detour and heuristic accuracy where independently established.
   A long path is not necessarily a hard search; equal-size maps are not
   interchangeable; obstacle density alone does not identify narrow passages.

The tool records numeric descriptors rather than inventing one combined
"difficulty score". Density labels use fixed boundaries of 0.2 and 0.6 and are
descriptive only. Size decade is floor(log10(V)); it is unavailable when the
actual free-node count is unknown. Source labels such as maze, room and sandstone
remain source metadata, not conclusions automatically proved by those numbers.

## What the five sources actually represent

| Source/family | Underlying problem | Structural interpretation | Query and evaluation caveats |
|---|---|---|---|
| DIMACS distance/time roads | Explicit directed weighted multigraph; geographical coordinates are additional metadata | Usually sparse; compare in/out degree, reciprocal arcs, weak/strong components and cost distribution | Preserve one-way arcs and units. Distance and travel time share topology but define different objectives. Regional road graphs overlap geographically. |
| MovingAI game, street, random, room, maze maps | Implicit regular 2D graph; land profile has 8 motions, costs 1 and sqrt(2), no corner cutting | Games mix open areas and enclosed spaces; streets constrain routes around buildings; generated maze/room/random families intentionally vary structure | The finite scenario set is not all possible pairs. Path-length buckets and source generators define its distribution. Street occupancy is not a directed road network. |
| MovingAI terrain | Regular 2D topology with terrain labels and nonuniform intended costs | All terrain is passable; labels describe regions rather than blocked cells | The original cost table is missing. Classify topology and label counts, but mark costs and scenario optima unavailable. Do not silently assign all cells cost 1. |
| MovingAI Warframe | Implicit regular 3D voxel graph | Predominantly open space with scattered obstacles; some maps include enclosed spaceship geometry | Source endpoint generation favours locations near obstacles. Use corrected scenarios from MovingAI; the older Monash mirror is not an additional independent collection. |
| Monash Descent | Implicit regular 3D graph from enclosed game levels | Rooms and branching interior corridors; the traversable interior is much smaller than the bounding volume | A huge bounding box does not imply an equally huge reachable component. Preserve source query selection and search-direction metadata. |
| Monash Sandstone | Implicit regular 3D graph from connected pore geometry | Winding, narrow, irregular passages; high solid fraction | `rev_voxel` lists free rather than blocked voxels. Uniform endpoint generation and the provided representative query set answer different evaluation questions. |
| Monash Industrial Plants | Implicit regular 3D graph for routing among equipment | Multiscale modules, safety zones and vertical routing structure | Endpoints near equipment attachments are application-specific. Do not equate this distribution with uniformly sampled drone navigation. |
| BARN | Geometry, occupancy and robot configuration-space representations of the same static scene | Cellular-automaton environments with cylinder obstacles, generated at varying parameters | Robot footprint, inflation and coordinate conversion change the effective free space. Supplied path difficulty metrics depend on that path; they are not whole-map invariants. |
| OMPL | A library/framework of problem definitions and generators, not one fixed finite graph dataset | Rn, SE(2), SE(3), articulated chains and constrained manifolds must be distinguished; workspace and configuration-space dimension differ | A sampled roadmap's V/E belong to that roadmap. Robot mesh collision, orientation and dynamics cannot be discarded when claiming compatibility with the source benchmark. |

Sources: [DIMACS catalog](https://www.diag.uniroma1.it/challenge9/download.shtml),
[MovingAI 2D](https://www.movingai.com/benchmarks/grids.html),
[terrain caveat](https://www.movingai.com/benchmarks/weighted/index.html),
[MovingAI 3D correction](https://www.movingai.com/benchmarks/voxels.html),
[Monash catalog](https://benchmarks.pathfinding.ai/3d-benchmarks/voxel-maps/),
[Monash paper](https://benchmarks.pathfinding.ai/assets/pdf/Nobes2023.pdf),
[BARN](https://www.cs.utexas.edu/~xiao/BARN/BARN.html),
[BARN generator and metrics](https://github.com/dperille/jackal-map-creation),
[OMPL demos](https://ompl.kavrakilab.org/demos.html).

### Specific source checks

- A regular grid embedded in 2D is not necessarily a planar graph: diagonal
  edges can cross without creating a vertex. The profiler does not infer
  graph-theoretic planarity from coordinates or the source family name.
- The Monash Warframe mirror identifies its scenarios as circa-2018 version 1;
  MovingAI documents regenerated problems after a 2019 movement-model error.
  Identical maps can therefore have different, incompatible scenario versions.
- Source Monash voxel coordinates can repeat. The profiler counts repeats while
  treating occupancy as a set, retaining the original byte hash and a separate
  duplicate-row measurement. It never counts repeated coordinates as new nodes.
- OMPL Circle Grid's default state space is SE(2), not simply R2. Its obstacle
  predicate depends only on XY; the XY free-volume marginal can be measured
  without claiming that its orientation-dependent distance metric is Euclidean.
- Hypercube's source comment describes the corridor in the opposite coordinate
  order from its implementation. The translated predicate follows the pinned
  code: high prefix coordinates, then one free coordinate, then low suffix
  coordinates. Tests exercise this distinction. The analytic free fraction in
  dimension n and width w is n*w^(n-1) - (n-1)*w^n for w <= 0.5.
- BARN's initial measured geometric view uses a point in XY and the obstacle
  envelope as explicit bounds. It is a derived view, not the Jackal's original
  C-space, simulator bounds, or complete sense-plan-act score. Source paths remain
  in row/column cell coordinates; no unverified world-coordinate transform is
  applied.

## Measurements and their meaning

Every measurement has `value` and `method`. A missing value is `null` with a
reason. Supported methods include `exact`, `header`, `analytic`, `uniform_sample`,
`source_path` and `source_unverified`. The report does not turn source claims,
samples, resource limits or missing data into measured zeroes.

**Explicit graphs:** V, arc count, in/out-degree distributions, weak/strong
component counts and largest-component fractions, reciprocal-arc fraction,
self loops, parallel arcs, cost distribution and negative/zero-cost counts.
Parallel arcs are preserved in degree and cost statistics; connectivity uses
the corresponding simple adjacency. Negative costs remain visible rather than
silently being converted into a Dijkstra-compatible input. Reciprocity is
measured on distinct directed adjacencies, including any self loops.

**Grids and voxels:** node slots, free nodes, directed legal-edge count, degree
distribution, components, largest-component fraction, dead-end fraction and
cell-center distance to the nearest blocked cell/boundary. The clearance
measurement is a geometric proxy in cells, not a robot clearance certificate or
an articulation/bottleneck statistic. Strict diagonal moves require every proper
side cell/voxel free. Therefore component membership agrees with cardinal
4/6-neighbor connectivity, although edge count and optimal costs differ.

**Continuous spaces:** finite V/E are inapplicable. Estimate free-volume fraction
with independent uniform points, record seed/sample size and a 95% Wilson
interval. A sample containing no valid point does not prove zero free volume,
disconnection or infeasibility. OMPL Circle Grid and Hypercube also have analytic
volume checks. Mesh/configuration-space topology remains explicitly unavailable
until an appropriate collision/kinematics adapter exists.

**Supplied queries:** count ordered pairs, duplicates, reversed pairs, normalized
Euclidean displacement within the map's bounding-box diagonal, and the source's
recorded costs. 3D scenario versions 1 and 2 are distinguished; source heuristic
and search-work ratios remain diagnostic source claims. Terrain source costs
are excluded. The characterization tool does not establish independent optimality
or reachability; those require a compatible independent oracle before planner
evaluation. DIMACS graph structure is profiled separately from query generation.

## Evaluation design that makes bias visible

1. **Define the target population first.** Report both the original source query
   distribution and an explicit family/map-balanced view. Equal family weights
   are a chosen evaluation target, not a universal unbiased estimate of production
   traffic. Never publish a single pooled leaderboard across incompatible models.
2. **Freeze inputs before candidate runs.** Map sampling uses a seeded hash of
   identity within each source/family. It does not use candidate success, baseline
   runtime, path cost or file order. Keep every unselected/resource-limited item
   in the catalog and disclose per-metric coverage.
3. **Use independent query axes.** For a balanced query view use fixed normalized
   displacement bins [0,.2,.4,.6,.8,1], retaining reachable, unreachable and unknown
   reference states separately. Empty bins stay empty. Do not condition primary
   selection on A* expansions: it would privilege that baseline's notion of
   difficulty. Detour C*/h and heuristic error require a valid independently
   established C* and a lower bound in the same cost units.
4. **Avoid duplicated evidence.** Track source versions, byte-identical payloads,
   mirrors, original/scaled geometry and distance/time variants with lineage.
   Use lineage for tuning/test splits and clustered uncertainty. Regional TIGER
   graphs additionally share a geographical dependency group; do not claim that
   overlapping regions are independent samples. Renamed or scaled maps that
   cannot be matched automatically require reviewed lineage metadata.
5. **Lock the comparison contract.** Match motion rules, corner policy, robot
   footprint, collision resolution, metric/objective, goal region, budget and
   preprocessing scope. The generated comparison key protects the known input
   semantics only; candidate-specific goal, heuristic and budget settings must
   still be locked by the actual benchmark protocol. Grid-optimal, any-angle and
   continuous costs require different references.
6. **Preserve failures.** Missing references, malformed files, download limits,
   errors, crashes and timeouts must be disclosed with denominators. Conditional
   latency over common successful queries is useful, but must accompany success
   coverage; it is not the unconditional performance of a planner.

The generated `cohort.json` proposes equal family then equal lineage weights
inside each comparison key, with variants sharing their lineage's weight.
Original/scaled Baldur's Gate collections share one originating-family weight. It
retains unmeasured maps and excludes the old Warframe mirror from the proposed
evaluation cohort. It is metadata for a future campaign, not evidence that all
those inputs have already been independently validated or benchmarked.

## Reproduction and interfaces

The implementation lives in `scripts/benchmark_datasets/`, with one CLI entry
point. No production planner API or existing v1/v2 benchmark contract changes.
It requires the repository's existing NumPy/SciPy development dependencies.

```bash
python scripts/analyze_benchmark_datasets.py catalog \
  --output benchmark-results/dataset-characterization/catalog.json
python scripts/analyze_benchmark_datasets.py profile \
  --catalog benchmark-results/dataset-characterization/catalog.json \
  --output benchmark-results/dataset-characterization \
  --per-family 3 --sample-size 16384
```

Defaults: seed 7, three map assets per source/family, 8,000,000 cells,
4,000,000 arcs, 256 MiB per downloaded/expanded asset and 4,096 uniform volume
samples. Generator definitions and the small OMPL configuration files are all
selected for semantic inspection. `--per-family 0` attempts the entire catalog;
limits still apply. The dated snapshot uses 16,384 volume samples. The size limit
is not an RSS limit: graph arrays, labels and distance transforms need additional
memory. No automatic resizing/downsampling changes a source map to make it fit.

Discovery follows API pagination and pins GitHub/Bitbucket file URLs to the
resolved revision. MovingAI/DIMACS URLs are mutable; keep the frozen catalog and
downloaded content hashes alongside results. Catalog discovery failures produce
a nonzero exit code. Profiling errors also produce a nonzero exit code; explicitly
recorded resource limits do not. Failed records are retained on resume. To retry
a failed record, use a new output directory with the same cache, retaining the
original attempt for audit. Changing profiler code, catalog entries or limits
changes its cache key. Reordering catalog entries may also invalidate that key;
the selected IDs remain unchanged.

Outputs under the ignored `benchmark-results/` directory:

- `catalog.json`: population inventory, source URLs, revisions, scope/exclusions.
- `profiles.json`: every catalog entry's status, classification, per-metric
  evidence, query audit and source hashes.
- `coverage.json`: source/family counts, metric availability, discovery failures,
  lineage and byte-identical groups.
- `cohort.json`: declared target weights and query selection policy.
- `report.md`: coverage first, then measured observations and missing coverage.

Regenerate the report offline with `report --profiles <file> --output <directory>`.
Use `stratify --queries <file> --output <file> --per-bin 20 --seed 7` to freeze
queries supplied as a JSON array containing `workload_id`, `map_id`,
`normalized_displacement`, and optional `reference_state` (`reachable`,
`unreachable`, `unknown`). Query IDs must be unique and already tied to the same
motion/cost profile; duplicate IDs fail rather than silently changing weights.

## Inventory snapshot

All 22 source/family collections were discovered without errors on 2026-10-08.

| Source | Catalog entries | Meaning |
|---|---:|---|
| DIMACS | 25 | 12 road networks x distance/time, plus Rome99 |
| BARN | 300 | Static cylinder-world scenes |
| MovingAI | 853 | 809 current 2D maps including 20 terrain maps, plus 44 3D maps |
| Monash | 90 | 46 new voxel maps plus 44 old Warframe mirror entries |
| OMPL | 29 | 25 OMPL.app configs plus 4 geometric demo definitions |
| Total | 1,297 | Entries are not 1,297 independent geometries |

This is full inventory coverage for the stated population. It is not full
structural measurement coverage. Resource-limited and unsupported topology
measurements remain explicit; quantitative conclusions must cite the actually
measured map/family subset in the generated coverage report.

## Measured snapshot and consequences

The seed-7 run selected 87 entries: three per map family, the sole Rome graph,
and all 29 small OMPL definitions/configurations. It processed 86 entries without
errors; the USA travel-time download exceeded 256 MiB. Another 1,210 entries
remain inventoried but unselected. "Processed" does not imply complete topology:
exact topology is available for 34 grids/voxels and four explicit graphs. Fourteen
selected voxel maps and two road graphs have header-only size measurements because
they exceed dense-cell/arc limits. Mesh problems and specialized generators have
explicitly unavailable topology. The run audited 161,460 supplied scenario rows
across 48 scenario files, plus three BARN supplied paths. It did not prove the
optimality or reachability of those queries.

| Named asset | Observation under the recorded profile | Evaluation consequence |
|---|---|---|
| DIMACS NY travel time | 264,346 vertices, 733,846 arcs; mean out-degree 2.776; one strong component; 3,746 parallel arcs | Sparse weighted multigraphs need their own stratum. Collapsing parallel arcs changes degree/cost data. |
| MovingAI `maze512-1-1.map` | Mean degree 2.000; 9.390% degree-one free nodes; median clearance 1 cell | Thin corridors and dead ends are visible structural features. |
| MovingAI `maze512-32-7.map` | Same 512x512 bounding shape, mean degree 7.803; no degree-one free nodes; median clearance 8 cells | Even the same source family and dimensions contain very different local geometry. |
| Monash `plant01.3dmap` | 2,416,298 free voxels, 3.193% blocked, 24 components; largest contains 85.240% of free voxels | Low obstacle density does not imply a single connected free space. Preserve query component membership when an oracle becomes available. |
| BARN `world_11.world` | XY point-envelope free fraction 91.589%, 95% interval [91.155%, 92.005%]; supplied path tortuosity 1.150 | Distinguish whole-scene samples from properties of the supplied path and from robot navigation scores. |
| OMPL Circle Grid | XY sampled free fraction 80.475%, analytic 80.365% | A volume estimate can be checked analytically but gives no connectivity certificate. |
| OMPL Hypercube, dimension 10 | Zero valid points in 16,384 uniform samples; analytic free fraction 9.1e-9; Wilson upper bound 0.0002344 | Zero hits must never become a claim that the problem is infeasible. |

`plant01` also contains 7,918 repeated coordinate rows. Occupancy-set semantics
preserve the free-node count while the duplicates remain visible as source data
quality evidence. The two selected mazes contain 15,850 and 4,940 supplied queries
respectively; pooling every query equally would already weight one map over three
times as heavily as the other. Report source-distribution and family/map-balanced
views separately, with their target distributions stated.

The 44 Monash Warframe entries are lineage-linked to MovingAI's 44 maps; they
are excluded from the proposed cohort because their older scenario version is
not independent evidence. Terrain rows retain their labels and query displacement
statistics, while cost comparisons remain unavailable without the original cost
table. OMPL inventory here is the explicitly named set above, not a claim that
all possible OMPL-generated problems or plugin-specific state spaces were measured.

## Validation snapshot

Eighteen focused tests pass, covering strict movement, graph direction and
parallel arcs, voxel duplicates, scenario discovery/versions, geometry transforms,
sampling reproducibility, unknown reachability, cache invalidation and coverage
weights. The new Python files pass Ruff checks and formatting; type checking
reports no errors (the standalone script check reports three missing SciPy stub
warnings). The native extension built successfully during repository validation.
The final `make test-all` run was not green: four failures, 257 passes and one
skip. The failures are the sampling-registry guard, direct-grid-import guard,
PRM* trace-test parameter compatibility, and the discrete registry smoke test
passing a regular grid to JPSW. These failures concern planner integration checks
outside this addition; planner/native/API files were left untouched.
