"""Contract tests for the production DynamicRRT3D planner."""

from __future__ import annotations

import subprocess
import sys

from pathplanning.sampling_based_planning.rrt_3d.dynamic_rrt_3d import (
    BruteForceNearestNodeIndex,
    DynamicRRT3D,
    KDTreeNearestNodeIndex,
)


def test_dynamic_rrt_import_is_headless_safe() -> None:
    """Importing the planner must not eagerly import plotting modules."""
    code = (
        "import importlib\n"
        "import sys\n"
        "before = set(sys.modules)\n"
        "importlib.import_module('pathplanning.sampling_based_planning.rrt_3d.dynamic_rrt_3d')\n"
        "loaded = set(sys.modules) - before\n"
        "bad = sorted(name for name in loaded if "
        "name == 'matplotlib' or name.startswith('matplotlib.') "
        "or name == 'pathplanning.sampling_based_planning.rrt_3d.plot_util_3d')\n"
        "if bad:\n"
        "    raise SystemExit('\\n'.join(bad))\n"
    )
    result = subprocess.run(
        [sys.executable, "-c", code], capture_output=True, text=True, check=False
    )
    assert result.returncode == 0, result.stdout + result.stderr


def test_dynamic_rrt_choose_target_is_deterministic_with_seed() -> None:
    """Seeded planners should generate the same sampled target sequence."""
    planner_a = DynamicRRT3D.with_seed(7)
    planner_b = DynamicRRT3D.with_seed(7)

    planner_a.init_rrt()
    planner_b.init_rrt()

    shared_nodes = [(3.0, 3.0, 1.0), (4.0, 9.0, 1.0), (8.0, 5.0, 2.0)]
    for node in shared_nodes:
        planner_a.add_node(planner_a.x0, node)
        planner_b.add_node(planner_b.x0, node)

    sequence_a = [planner_a.choose_target() for _ in range(25)]
    sequence_b = [planner_b.choose_target() for _ in range(25)]

    assert sequence_a == sequence_b


def test_dynamic_rrt_nearest_backends_are_consistent() -> None:
    """Brute-force and KDTree nearest backends should agree on nearest node."""
    brute = DynamicRRT3D.with_seed(0, nearest_index=BruteForceNearestNodeIndex())
    kd_tree = DynamicRRT3D.with_seed(0, nearest_index=KDTreeNearestNodeIndex(rebuild_threshold=2))

    for planner in (brute, kd_tree):
        planner.init_rrt()
        for node in [
            (3.0, 1.0, 2.0),
            (6.0, 3.0, 2.5),
            (9.0, 4.0, 1.5),
            (11.0, 6.0, 3.0),
        ]:
            planner.add_node(planner.x0, node)

    target = (7.8, 3.4, 2.7)
    assert brute.nearest(target) == kd_tree.nearest(target)
