# Recorded planner run inputs

`ost102d.map` is the MovingAI DAO map used for the A* trace. MovingAI distributes these benchmark maps under the Open Data Commons Attribution License (ODC-By); see the [official benchmark page](https://www.movingai.com/benchmarks/grids.html) and cite Nathan Sturtevant, *Benchmarks for Grid-Based Pathfinding*, IEEE Transactions on Computational Intelligence and AI in Games, 2012.

`barn_world_236.json` contains only the obstacle cylinders and query derived from BARN `world_236.world`, not the original simulator world file. Its JSON metadata records the immutable source revision, URL, source-file SHA-256, geometry extraction, point-robot assumption, and query derivation. See the [BARN dataset page](https://www.cs.utexas.edu/~xiao/BARN/BARN.html) and cite Perille et al., *Benchmarking Robot Manipulation and Navigation with the BARN Challenge*, 2020. The BARN page requests citation but does not state a data license; source attribution and provenance are retained here.

`dstar_lite_obstacle_insertion.json` defines a small illustrative graph. It is a reproducible package run, not an external benchmark instance.

From the repository root, regenerate all three saved traces with:

```bash
.venv/bin/python -m scripts.generate_planner_demo_runs
```
