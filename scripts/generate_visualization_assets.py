"""Generate README and documentation figures with the current visualization API."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")

from matplotlib.lines import Line2D
from matplotlib.patches import Patch
import numpy as np

from pathplanning import TraceOptions, plan_continuous, plan_discrete
from pathplanning.core.contracts import ContinuousProblem, DiscreteProblem, GoalState
from pathplanning.spaces import AABB, ContinuousSpace3D, Grid2DSamplingSpace, Grid2DSearchSpace
from pathplanning.spaces import Sphere as SpaceSphere
from pathplanning.viz import render_result, scene_from_problem
from pathplanning.viz.replay import ReplayController

ROOT = Path(__file__).resolve().parents[1]
OUTPUT_DIR = ROOT / "assets" / "images"


def _handles(kind: str) -> list[Any]:
    handles = [Patch(facecolor="#667085", alpha=0.5, label="Obstacle")]
    if kind == "search":
        handles.extend(
            [
                Line2D([], [], marker="o", linestyle="", color="#12b76a", label="Visited"),
                Line2D([], [], marker="o", linestyle="", color="#f79009", label="Frontier"),
            ]
        )
    elif kind == "bidirectional":
        handles.extend(
            [
                Line2D(
                    [], [], marker="o", linestyle="", color="#12b76a", label="Start-side visited"
                ),
                Line2D(
                    [], [], marker="o", linestyle="", color="#2e90fa", label="Goal-side visited"
                ),
                Line2D([], [], marker="o", linestyle="", color="#f79009", label="Start frontier"),
                Line2D([], [], marker="o", linestyle="", color="#7a5af8", label="Goal frontier"),
            ]
        )
    elif kind == "sampling-bidirectional":
        handles.extend(
            [
                Line2D([], [], color="#12b76a", linewidth=1.5, label="Start tree"),
                Line2D([], [], color="#2e90fa", linewidth=1.5, label="Goal tree"),
                Line2D([], [], marker="o", linestyle="", color="#98a2b3", label="Samples"),
            ]
        )
    else:
        handles.extend(
            [
                Line2D([], [], color="#12b76a", linewidth=1.5, label="Tree"),
                Line2D([], [], marker="o", linestyle="", color="#98a2b3", label="Samples"),
            ]
        )
    handles.extend(
        [
            Line2D([], [], color="#f04438", linewidth=2.5, label="Path"),
            Line2D(
                [],
                [],
                marker="o",
                linestyle="",
                color="#12b76a",
                markeredgecolor="black",
                label="Start",
            ),
            Line2D(
                [],
                [],
                marker="*",
                linestyle="",
                color="#f04438",
                markeredgecolor="black",
                label="Goal",
            ),
        ]
    )
    return handles


def _save_snapshot(
    problem: DiscreteProblem[Any] | ContinuousProblem[Any],
    result: Any,
    *,
    filename: str,
    title: str,
    kind: str,
) -> None:
    if not result.success:
        raise RuntimeError(f"planner did not find a path for {filename}: {result.stop_reason}")
    if result.trace is None:
        raise RuntimeError(f"trace missing for visualization asset {filename}")

    scene = scene_from_problem(problem)
    scene.title = title
    renderer = render_result(scene, result)
    controller = ReplayController(result.trace)
    snapshot = controller.seek(controller.length)
    renderer.show_state(snapshot, result.trace)

    figure = renderer.figure
    figure.set_size_inches(10.5, 6.5)
    axes = renderer.ax
    axes.set_title(title, loc="center", fontsize=16, fontweight="bold", pad=16)
    axes.grid(True, color="#eaecf0", linewidth=0.7, alpha=0.8)
    axes.set_axisbelow(True)
    for spine in axes.spines.values():
        spine.set_color("#d0d5dd")

    figure.subplots_adjust(left=0.10, right=0.97, top=0.88, bottom=0.18)
    figure.legend(
        handles=_handles(kind),
        loc="lower center",
        bbox_to_anchor=(0.53, 0.045),
        ncol=4,
        frameon=False,
        fontsize=9,
    )
    figure.text(
        0.10,
        0.025,
        f"{result.nodes:,} nodes · {result.iters:,} iterations",
        fontsize=9,
        color="#667085",
    )
    target = OUTPUT_DIR / filename
    target.parent.mkdir(parents=True, exist_ok=True)
    renderer.save(target, dpi=170, bbox_inches="tight", facecolor="white")
    figure.clear()
    print(f"wrote {target.relative_to(ROOT)}")


def _search_problem() -> DiscreteProblem[tuple[int, int]]:
    width, height = 48, 32
    obstacles = {(22, y) for y in range(1, height - 1) if not 13 <= y <= 18} | {
        (35, y) for y in range(7, height - 1) if not 21 <= y <= 25
    }
    graph = Grid2DSearchSpace(width=width, height=height, obstacles=obstacles)
    return DiscreteProblem(graph=graph, start=(3, 4), goal=(44, 27))


def _sampling_2d_problem() -> ContinuousProblem[np.ndarray]:
    space = Grid2DSamplingSpace(
        x_range=(0.0, 30.0),
        y_range=(0.0, 18.0),
        obs_rectangle=((7.0, 0.0, 2.0, 12.0), (14.0, 6.0, 2.5, 12.0), (22.0, 0.0, 2.0, 11.0)),
        obs_circle=((18.5, 3.0, 1.4),),
        collision_step=0.2,
    )
    return ContinuousProblem(
        space=space,
        start=np.array([2.0, 2.0]),
        goal=GoalState(np.array([28.0, 16.0]), radius=0.0),
    )


def _sampling_3d_problem() -> ContinuousProblem[np.ndarray]:
    space = ContinuousSpace3D(
        lower_bound=np.array([0.0, 0.0, 0.0]),
        upper_bound=np.array([12.0, 12.0, 5.0]),
        aabbs=(AABB(np.array([5.0, 3.0, 0.0]), np.array([7.0, 9.0, 3.2])),),
        spheres=(SpaceSphere(np.array([3.0, 8.0, 2.0]), 1.0),),
    )
    return ContinuousProblem(
        space=space,
        start=np.array([1.0, 1.0, 1.0]),
        goal=GoalState(np.array([11.0, 11.0, 1.0])),
    )


def main() -> None:
    trace = TraceOptions()
    search = _search_problem()
    for planner, filename, title in (
        ("astar", "astar-2d.png", "A* · grid search"),
        (
            "bidirectional_astar",
            "bidirectional-astar-2d.png",
            "Bidirectional A* · two search fronts",
        ),
    ):
        result = plan_discrete(search, planner=planner, trace=trace)
        _save_snapshot(
            search,
            result,
            filename=filename,
            title=title,
            kind="bidirectional" if planner == "bidirectional_astar" else "search",
        )

    sampling_2d = _sampling_2d_problem()
    rrt = plan_continuous(
        sampling_2d,
        planner="rrt",
        params={"max_iters": 3_000, "step_size": 1.1, "goal_sample_rate": 0.12},
        seed=7,
        trace=trace,
    )
    _save_snapshot(
        sampling_2d,
        rrt,
        filename="rrt-2d.png",
        title="RRT · continuous 2D planning",
        kind="sampling",
    )

    connect_2d = plan_continuous(
        sampling_2d,
        planner="rrt_connect",
        params={"max_iters": 1_000, "step_size": 1.0, "goal_sample_rate": 0.12},
        seed=7,
        trace=trace,
    )
    _save_snapshot(
        sampling_2d,
        connect_2d,
        filename="rrt-connect-2d.png",
        title="RRT-Connect · bidirectional tree growth",
        kind="sampling-bidirectional",
    )

    sampling_3d = _sampling_3d_problem()
    connect_3d = plan_continuous(
        sampling_3d,
        planner="rrt_connect",
        params={"max_iters": 1_500, "step_size": 0.7, "goal_sample_rate": 0.15},
        seed=7,
        trace=trace,
    )
    _save_snapshot(
        sampling_3d,
        connect_3d,
        filename="rrt-connect-3d.png",
        title="RRT-Connect · 3D scene",
        kind="sampling-bidirectional",
    )


if __name__ == "__main__":
    main()
