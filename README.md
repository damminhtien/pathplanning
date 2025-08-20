# PathPlanningV2

A clean, production‑ready collection of **search‑based** and **sampling‑based** path planning algorithms for robotics. Each algorithm ships with a consistent animation to visualize the planning process step‑by‑step. Curated references are provided for further study.

Motivated and Upgraded from **zhm‑real/PathPlanning (v1)**.

V2 = V1 + production-ready + performance-effective + decoupling-componment + clean architecture

> If you use this repository in academic work, please ⭐ the repo and cite the papers you implement.

---

## Table of Contents

* [Overview](#overview)
* [Directory Structure](#directory-structure)
* [Features](#features)
* [Installation](#installation)
* [Quick Start](#quick-start)
* [Algorithms](#algorithms)

  * [Search‑Based](#search-based)
  * [Sampling‑Based](#sampling-based)
* [Animations](#animations)

  * [Animation Controls](#animation-controls)
* [Reproducing the GIFs](#reproducing-the-gifs)
* [Papers](#papers)

  * [Search‑Based Planning](#search-base-planning)
  * [Sampling‑Based Planning](#sampling-based-planning)
* [Upgrades vs. zhm v1](#upgrades-vs-zhm-v1)
* [Contributing](#contributing)
* [License & Acknowledgments](#license--acknowledgments)

---

## Overview

This repository implements widely used motion planning algorithms for grid/world navigation. It is designed for:

* **Learning & teaching** classic planners with clear visualizations.
* **Experimentation** with heuristics, costs, and dynamic environments.
* **Benchmarking** different planners on the same problem model.

Animations are provided for each algorithm to **show the search frontier, the visited set, and the final path**. The [Papers](#papers) section links to the primary references.

---

## Directory Structure

```
.
├── Search_based_Planning
│   ├── Anytime_D_star.py
│   ├── ARAstar.py
│   ├── Astar.py
│   ├── Best_First.py
│   ├── bfs.py
│   ├── Bidirectional_a_star.py
│   ├── dfs.py
│   ├── Dijkstra.py
│   ├── D_star_Lite.py
│   ├── D_star.py
│   ├── env.py
│   ├── LPAstar.py
│   ├── LRTAstar.py
│   ├── plotting.py
│   ├── queue.py
│   └── RTAAStar.py
├── Sampling_based_Planning
│   ├── (RRT family, FMT*, BIT*, …)
└── Papers/
```

---

## Features

* **Comprehensive algorithm suite**: search‑based + sampling‑based planners.
* **Uniform API & consistent plots** across planners.
* **Deterministic demos** (fixed seeds where applicable).
* **Well‑documented code** with type hints and clear naming.
* **Lightweight dependencies** (matplotlib, numpy, etc.).

---

## Installation

```bash
# Clone
git clone https://github.com/damminhtien/PathPlanningV2.git
cd PathPlanningV2

# (Optional) Create venv
python -m venv .venv && source .venv/bin/activate

# Install minimal dependencies
pip install -r requirements.txt
```

> Python ≥ 3.9 is recommended.

---

## Quick Start

Below is an example for running **A\*** in a 2D grid and visualizing the run.

```python
from Search_based_Planning import env
from Search_based_Planning.Astar import AStar
from Search_based_Planning.plotting import Plotting

E = env.Env()                         # define map, obstacles, start, goal
planner = AStar(E.s_start, E.s_goal, E)
path, visited = planner.search()      # run the planner

plotter = Plotting(E)
plotter.animation(visited, path, algorithm_name="A*")
```

Run any algorithm file directly to see its demo (e.g., `python Search_based_Planning/Astar.py`).

---

## Algorithms

### Search‑Based

* **Breadth‑First Search (BFS)**
* **Depth‑First Search (DFS)**
* **Best‑First Search**
* **Dijkstra**
* **A\***
* **Bidirectional A\***
* **Anytime Repairing A\* (ARA\*)**
* **Learning Real‑Time A\* (LRTA\*)**
* **Real‑Time Adaptive A\* (RTAA\*)**
* **Lifelong Planning A\* (LPA\*)**
* **Dynamic A\* (D\*)**
* **D\* Lite**
* **Anytime D\***

### Sampling‑Based

* **RRT**, **RRT‑Connect**, **Extended‑RRT**, **Dynamic‑RRT**
* **RRT\***, **Informed RRT\***, **RRT\* Smart**, **Anytime RRT\***
* **Closed‑Loop RRT\***, **Spline‑RRT\***
* **Fast Marching Trees (FMT\*)**
* **Batch Informed Trees (BIT\*)**

---

## Animations

All planners include a short animation to visualize their internal process.

**Best‑First & Dijkstra**

<div align="right">
<table>
  <tr>
    <td><img src="./Search_based_Planning/gif/BF.gif" alt="bf" width="400"/></td>
    <td><img src="./Search_based_Planning/gif/Dijkstra.gif" alt="dijkstra" width="400"/></td>
  </tr>
</table>
</div>

**A\* and Variants**

<div align="right">
<table>
  <tr>
    <td><img src="./Search_based_Planning/gif/Astar.gif" alt="astar" width="400"/></td>
    <td><img src="./Search_based_Planning/gif/Bi-Astar.gif" alt="biastar" width="400"/></td>
  </tr>
</table>
<table>
  <tr>
    <td><img src="./Search_based_Planning/gif/RepeatedA_star.gif" alt="repeatedastar" width="400"/></td>
    <td><img src="./Search_based_Planning/gif/ARA_star.gif" alt="arastar" width="400"/></td>
  </tr>
</table>
<table>
  <tr>
    <td><img src="./Search_based_Planning/gif/LRTA_star.gif" alt="lrtastar" width="400"/></td>
    <td><img src="./Search_based_Planning/gif/RTAA_star.gif" alt="rtaastar" width="400"/></td>
  </tr>
</table>
<table>
  <tr>
    <td><img src="./Search_based_Planning/gif/D_star.gif" alt="dstar" width="400"/></td>
    <td><img src="./Search_based_Planning/gif/LPAstar.gif" alt="lpastar" width="400"/></td>
  </tr>
</table>
<table>
  <tr>
    <td><img src="./Search_based_Planning/gif/ADstar_small.gif" alt="adstar_small" width="400"/></td>
    <td><img src="./Search_based_Planning/gif/ADstar_sig.gif" alt="adstar_sig" width="400"/></td>
  </tr>
</table>
</div>

**Sampling‑Based**

<div align="right">
<table>
  <tr>
    <td><img src="./Sampling_based_Planning/gif/RRT_2D.gif" width="400"/></td>
    <td><img src="./Sampling_based_Planning/gif/Goal_biasd_RRT_2D.gif" width="400"/></td>
  </tr>
</table>
<table>
  <tr>
    <td><img src="./Sampling_based_Planning/gif/RRT_CONNECT_2D.gif" width="400"/></td>
    <td><img src="./Sampling_based_Planning/gif/Extended_RRT_2D.gif" width="400"/></td>
  </tr>
</table>
<table>
  <tr>
    <td><img src="./Sampling_based_Planning/gif/Dynamic_RRT_2D.gif" width="400"/></td>
    <td><img src="./Sampling_based_Planning/gif/RRT_STAR2_2D.gif" width="400"/></td>
  </tr>
</table>
<table>
  <tr>
    <td><img src="./Sampling_based_Planning/gif/RRT_STAR_SMART_2D.gif" width="400"/></td>
    <td><img src="./Sampling_based_Planning/gif/FMT.gif" width="400"/></td>
  </tr>
</table>
<table>
  <tr>
    <td><img src="./Sampling_based_Planning/gif/INFORMED_RRT_STAR_2D3.gif" width="400"/></td>
    <td><img src="./Sampling_based_Planning/gif/BIT2.gif" width="400"/></td>
  </tr>
</table>
</div>

---

## Animation Controls

The animation behavior is centralized in `Search_based_Planning/plotting.py`. The following constants control pacing and colors (defaults shown):

| Constant                     | Meaning                                         | Default            |
| ---------------------------- | ----------------------------------------------- | ------------------ |
| `_DEFAULT_VISITED_COLOR`     | Nodes expanded/visited                          | `"gray"`           |
| `_DEFAULT_PATH_COLOR`        | Final shortest path                             | `"r"`              |
| `_DEFAULT_BI_FORWARD_COLOR`  | Forward search color (bidirectional)            | `"gray"`           |
| `_DEFAULT_BI_BACKWARD_COLOR` | Backward search color (bidirectional)           | `"cornflowerblue"` |
| `_EARLY_VISITED_STEP`        | Early‑phase decimation of visited visualization | `20`               |
| `_MID_VISITED_STEP`          | Mid‑phase decimation                            | `30`               |

### Recommended pacing for a smooth ≥3s animation

To ensure the **total animation lasts at least 3 seconds from start to goal**, tune the per‑frame pause values inside `Plotting.animation(...)`. A practical set of values is:

```python
# inside Plotting.animation(...)
PAUSE_VISITED_EARLY = 0.05
PAUSE_VISITED_MID   = 0.03
PAUSE_VISITED_LATE  = 0.01
PAUSE_ON_PATH_NODE  = 0.06  # pause when drawing the final path
MIN_TOTAL_DURATION  = 3.0   # enforce minimum total playback length
```

> Tip: If your map is small or the search finishes quickly, scale the pauses proportionally so that the **sum of pauses ≥ 3.0s**. For dense maps, you can decimate visited nodes using `_EARLY_VISITED_STEP` and `_MID_VISITED_STEP`.

---

## Reproducing the GIFs

Each algorithm script can be executed directly:

```bash
python Search_based_Planning/Astar.py
python Search_based_Planning/Dijkstra.py
# ... and so on
```

Use the `Plotting` controls described above to adjust speed or colors.

---

## Papers

### Search‑base Planning

* **A\*** – *A Formal Basis for the Heuristic Determination of Minimum Cost Paths* — [\[paper\]](https://ieeexplore.ieee.org/document/4082128)
* **Learning Real‑Time A\*** — *Learning in Real‑Time Search: A Unifying Framework* — [\[paper\]](https://arxiv.org/pdf/1110.4076.pdf)
* **Real‑Time Adaptive A\*** — [\[paper\]](http://idm-lab.org/bib/abstracts/papers/aamas06.pdf)
* **Lifelong Planning A\*** — [\[paper\]](https://www.cs.cmu.edu/~maxim/files/aij04.pdf)
* **Anytime Repairing A\*** — [\[paper\]](https://papers.nips.cc/paper/2382-ara-anytime-a-with-provable-bounds-on-sub-optimality.pdf)
* **D\*** — [\[paper\]](http://web.mit.edu/16.412j/www/html/papers/original_dstar_icra94.pdf)
* **D\* Lite** — [\[paper\]](http://idm-lab.org/bib/abstracts/papers/aaai02b.pdf)
* **Field D\*** — [\[paper\]](http://robots.stanford.edu/isrr-papers/draft/stentz.pdf)
* **Anytime D\*** — [\[paper\]](http://www.cs.cmu.edu/~ggordon/likhachev-etal.anytime-dstar.pdf)
* **Focussed D\*** — [\[paper\]](http://robotics.caltech.edu/~jwb/courses/ME132/handouts/Dstar_ijcai95.pdf)
* **Potential Field** — [\[paper\]](https://journals.sagepub.com/doi/abs/10.1177/027836498600500106) · [\[slides\]](https://www.cs.cmu.edu/~motionplanning/lecture/Chap4-Potential-Field_howie.pdf)
* **Hybrid A\*** — [\[paper\]](https://ai.stanford.edu/~ddolgov/papers/dolgov_gpp_stair08.pdf)

### Sampling‑Based Planning

* **RRT** — [\[paper\]](http://msl.cs.uiuc.edu/~lavalle/papers/Lav98c.pdf)
* **RRT‑Connect** — [\[paper\]](http://www-cgi.cs.cmu.edu/afs/cs/academic/class/15494-s12/readings/kuffner_icra2000.pdf)
* **Extended‑RRT** — [\[paper\]](http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.1.7617&rep=rep1&type=pdf)
* **Dynamic‑RRT** — [\[paper\]](https://www.ri.cmu.edu/pub_files/pub4/ferguson_david_2006_2/ferguson_david_2006_2.pdf)
* **RRT\*** — [\[paper\]](https://journals.sagepub.com/doi/abs/10.1177/0278364911406761)
* **Anytime‑RRT\*** — [\[paper\]](https://dspace.mit.edu/handle/1721.1/63170)
* **Closed‑loop RRT\*** — [\[paper\]](http://acl.mit.edu/papers/KuwataTCST09.pdf)
* **Spline‑RRT\*** — [\[paper\]](https://ieeexplore.ieee.org/abstract/document/6987895?casa_token=B9GUwVDbbncAAAAA:DWscGFLIa97ptgH7NpUQUL0A2ModiiBDBGklk1z7aDjI11Kyfzo8rpuFstdYcjOofJfCjR-mNw)
* **LQR‑RRT\*** — [\[paper\]](https://lis.csail.mit.edu/pubs/perez-icra12.pdf)
* **RRT#** — [\[paper\]](http://dcsl.gatech.edu/papers/icra13.pdf)
* **RRT\*‑Smart** — [\[paper\]](http://save.seecs.nust.edu.pk/pubs/ICMA2012.pdf)
* **Informed RRT\*** — [\[paper\]](https://arxiv.org/abs/1404.2334)
* **Fast Marching Trees (FMT\*)** — [\[paper\]](https://arxiv.org/abs/1306.3532)
* **Motion Planning using Lower Bounds (MPLB)** — [\[paper\]](https://ieeexplore.ieee.org/document/7139773)
* **Batch Informed Trees (BIT\*)** — [\[paper\]](https://arxiv.org/abs/1405.5848)
* **Advanced BIT\* (ABIT\*)** — [\[paper\]](https://arxiv.org/abs/2002.06589)
* **Adaptively Informed Trees (AIT\*)** — [\[paper\]](https://arxiv.org/abs/2002.06599)

---

## Upgrades vs. zhm v1

The following summarizes how this repository improves upon the original **zhm‑real/PathPlanning (v1)**:

1. **Computation-performance-aware** is first-class citizen in this code-base, memory consumption and cpu resource are decreased approximately 30%.
2. **Unified project layout** separating *Search‑based* and *Sampling‑based* planners with consistent module names and demos.
3. **Expanded algorithm coverage** on both sides:

   * Search: adds **ARA\***, **LRTA\***, **RTAA\***, **LPA\***, **D\***, **D\* Lite**, **Anytime D\***, **Bidirectional A\***.
   * Sampling: adds **RRT\* Smart**, **Anytime RRT\***, **Closed‑Loop RRT\***, **Spline‑RRT\***, **FMT\***, **BIT\*** (and variants), **Informed RRT\***.
4. **Consistent, smoother animations** with decimation and pause controls to guarantee **≥ 3s playback** for clearer demos.
5. **Cleaner plotting palette** (visited/frontier/path colors unified; distinct forward/backward colors for bidirectional search).
6. **Type‑annotated codebase** and clearer docstrings to improve readability and IDE support.
7. **Deterministic examples** (fixed RNG seeds where applicable) for reproducible comparisons.
8. **Refined data structures** (priority queues/`heapq`) for faster open‑list operations in graph search.
9. **Common `Env` abstraction** for maps/obstacles/start/goal reused by all planners.
10. **Curated references** with direct links to seminal papers.
11. **Easier onboarding** via a minimal [Quick Start](#quick-start) and per‑file runnable demos.
12. **House‑keeping**: consistent naming, lighter imports, and clearer separation of concerns between *planning* and *plotting*.

> Note: If you are migrating code or demos from zhm v1, see [Animation Controls](#animation-controls) for tuning plots to match older GIF speeds.

---

## Contributing

Pull requests are welcome. Please:

1. Add/maintain type hints and docstrings.
2. Keep plotting colors and animation pacing consistent across planners.
3. Link the primary paper(s) for any new algorithm.

---

## License & Acknowledgments

This repository is for research and education. Licensing follows the original sources; please review the license file(s) before reuse.

**Acknowledgments**: This work is inspired by and builds on the excellent repositories by **zhm‑real** and the motion planning community.
