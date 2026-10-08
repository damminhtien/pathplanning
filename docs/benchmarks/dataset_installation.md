# Installing benchmark datasets

Install the surveyed dataset collection into `benchmark-results/datasets/`:

```bash
python3.12 scripts/install_benchmark_datasets.py install \
  --catalog benchmark-results/dataset-characterization/catalog.json
python3.12 scripts/install_benchmark_datasets.py status
python3.12 scripts/install_benchmark_datasets.py verify
```

The installer reuses the survey download cache in
`benchmark-results/dataset-characterization/cache/`. If that catalog is absent,
`install` discovers the sources and saves a copy beside the installed files.
Downloads stream to disk, retry temporary network failures, and retain complete
files between runs. The defaults allow four downloads at once, cap one download
at 8 GiB and one expanded file at 16 GiB, and preserve at least 2 GiB of free
space. Change these limits with `--workers`, `--max-asset-gib`,
`--max-expanded-gib`, and `--min-free-gib` when needed.

`index.json` records each dataset's source URL, revision or version, lineage,
installed path, role, size, SHA-256, status, and benchmark support. It also
records source-level attribution and assets shared by multiple dataset entries.
The copied `catalog.json` retains the surveyed inventory and original source
links. Both files and all downloaded data are under the ignored
`benchmark-results/` directory.

| Source | Dataset entries | Installed files | Layout and current support |
|---|---:|---:|---|
| DIMACS | 25 | 37 | 25 graph files and 12 shared coordinate files under `dimacs/`; registered for graph-format adapters. |
| BARN | 300 | 600 | 300 `.world` files and their 300 supplied `.npy` paths under `barn/`; registered data only. |
| MovingAI | 853 | 1,706 | 789 land maps, 20 terrain maps, 44 voxel maps and linked scenarios under `movingai-v2/`, `movingai-terrain/`, and `movingai-3d/`. The current shortest-path runner supports the 789 2D land maps. |
| Monash | 90 | 179 | 90 voxel maps and 89 linked scenarios under `monash/`; one map has no scenario linked by the source. |
| OMPL / OMPL.app | 29 | 105 | 99 pinned OMPL.app resources, four demo source files, `KinematicChain.h`, and `floor.ppm` under `ompl/`; configs and referenced resources are registered, but no OMPL, Gazebo, or ROS software is installed. |
| **Total** | **1,297** | **2,627** | One regional DIMACS coordinate file is shared by its distance and travel-time graphs. |

MovingAI land maps and scenario files are placed beside one another in each
family directory so the existing parser resolves the scenario's map reference.
Terrain and voxel files are kept outside `movingai-v2/` to keep them out of the
land profile. The older Warframe mirrors in Monash keep their old version and
lineage metadata; they are not silently treated as independent new maps.

Check installation progress with `status`. It counts catalog entries that are
installed, not yet installed, missing a local file, missing a source-linked
asset, or in error. Run `verify` after installation or when files may have
changed: it recomputes SHA-256 hashes and checks MovingAI map/scenario pairs,
Monash scenario bounds, and mesh or image references in OMPL.app configurations.
The Monash map without a linked scenario is retained as an explicit source gap.

For a fresh checkout, the installer discovers sources if no catalog is passed
and no frozen survey catalog exists at the default sibling path. A source subset
can be installed or verified with `--sources dimacs movingai`. A successful
install records failures per asset and continues with other files; it exits
nonzero if a selected required asset failed. The `status` and `verify` commands
also return nonzero when discovery or verification finds an error.

## Prepare the MovingAI shortest-path benchmark

Build the native extension, then prepare from the installed land maps. No manual
download or extraction step is needed:

```bash
python3.12 setup.py build_ext --inplace \
  --build-temp /tmp/pathplanning-native-build/temp \
  --build-lib /tmp/pathplanning-native-build/lib
python3.12 scripts/benchmark_shortest_path.py prepare \
  --dataset-root benchmark-results/datasets/movingai-v2 \
  --profile pilot \
  --manifest benchmark-results/pilot_manifest.json
```

The other collections are installed and characterized, but their formats are
not yet accepted by this MovingAI land runner. Dataset installation does not run
planners or create benchmark measurements. See
[`shortest_path_benchmark_reproduction.md`](../shortest_path_benchmark_reproduction.md)
for validation and campaign commands.

Original source pages and their terms are preserved in `index.json`: [DIMACS
Challenge 9](https://www.diag.uniroma1.it/challenge9/download.shtml), [BARN
benchmark](https://www.cs.utexas.edu/~xiao/BARN/BARN.html), [MovingAI
benchmarks](https://www.movingai.com/benchmarks/grids.html), [Monash voxel
benchmarks](https://benchmarks.pathfinding.ai/3d-benchmarks/voxel-maps/), and
[OMPL demos](https://ompl.kavrakilab.org/demos.html). Keep those source
attributions with any results that use the data.
