#!/usr/bin/env python3
"""Render real MovingAI map/scenario queries for the README gallery."""

from __future__ import annotations

import argparse
from collections import defaultdict
import json
from pathlib import Path
import sys
from typing import Any

import matplotlib

matplotlib.use("Agg")

from matplotlib.colors import ListedColormap, to_rgba
from matplotlib.lines import Line2D
from matplotlib.patches import Patch
import matplotlib.pyplot as plt
import numpy as np
import scienceplots  # noqa: F401  # Register the package's Matplotlib styles.

plt.style.use(["science", "no-latex"])

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from pathplanning.api import plan_discrete
from pathplanning.core.contracts import DiscreteProblem
from pathplanning.core.trace import TraceOptions
from pathplanning.viz.replay import ReplayController
from scripts.shortest_path_benchmark.workloads import (
    MOVEMENT_PROFILE,
    MovingAIScenario,
    parse_scenario,
)

ALGORITHM_LABELS = {
    "astar": "A*",
    "bidirectional_astar": "Bidirectional A*",
    "bidirectional_dijkstra": "Bidirectional Dijkstra",
    "dijkstra": "Dijkstra",
    "greedy_best_first": "Greedy best-first",
    "weighted_astar": "Weighted A* (w = 1.5)",
}
FAMILY_LABELS = {
    "maze": "MovingAI maze",
    "room": "MovingAI rooms",
    "dao": "MovingAI DAO",
    "street": "MovingAI street",
    "sc1": "MovingAI StarCraft",
}
SCENES = (
    ("maze", "astar", {}, "movingai-maze-scenario.png"),
    ("room", "bidirectional_astar", {}, "movingai-room-scenario.png"),
    ("dao", "weighted_astar", {"weight": 1.5}, "movingai-dao-scenario.png"),
    ("street", "astar", {}, "movingai-street-scenario.png"),
    ("sc1", "astar", {}, "movingai-sc1-scenario.png"),
)
EXPLORED_COLORS = ("#12b76a", "#2e90fa")
FRONTIER_COLORS = ("#f79009", "#7a5af8")
PATH_COLOR = "#f04438"
START_COLOR = "#12b76a"
GOAL_COLOR = "#f04438"
DEFAULT_CAMPAIGN = Path("benchmark-results/pilot/runs.jsonl")
DEFAULT_DATASET_ROOT = Path("benchmark-results/datasets/movingai-v2")
DEFAULT_OUTPUT_DIR = Path("assets/images")


def _candidate_rows(campaign_path: Path) -> dict[str, list[dict[str, Any]]]:
    rows: dict[str, list[dict[str, Any]]] = defaultdict(list)
    with campaign_path.open(encoding="utf-8") as campaign:
        for line in campaign:
            record = json.loads(line)
            inputs = record.get("input", {})
            outcome = record.get("outcome", {})
            if (
                record.get("pass") != "work"
                or record.get("phase") != "measured"
                or record.get("variant_name") != "astar"
                or record.get("status") != "ok"
                or not outcome.get("path_valid")
                or not outcome.get("path_present")
            ):
                continue
            if inputs.get("movement_profile") != MOVEMENT_PROFILE:
                continue
            rows[str(inputs.get("family", ""))].append(record)
    return rows


def _select_case(rows_by_family: dict[str, list[dict[str, Any]]], family: str) -> dict[str, Any]:
    rows = rows_by_family.get(family, [])
    if not rows:
        raise ValueError(f"The campaign has no valid measured A* workload for family {family!r}")
    ordered = sorted(
        rows,
        key=lambda row: (
            int(row["outcome"].get("path_length", 0)),
            str(row.get("workload_id", "")),
        ),
    )
    return ordered[len(ordered) // 2]


def _resolve_scenario(record: dict[str, Any], dataset_root: Path) -> MovingAIScenario:
    root = dataset_root.resolve(strict=True)
    inputs = record["input"]
    relative_map = Path(inputs["map_path"])
    if relative_map.is_absolute() or ".." in relative_map.parts:
        raise ValueError(f"Map path escapes the dataset root: {relative_map}")
    map_path = (root / relative_map).resolve(strict=True)
    if not map_path.is_relative_to(root):
        raise ValueError(f"Map path escapes the dataset root: {relative_map}")
    scenario_path = map_path.with_name(f"{map_path.name}.scen")
    scenarios = parse_scenario(scenario_path, root, family=str(inputs["family"]))
    wanted_line = int(inputs["scenario_line"])
    scenario = next((item for item in scenarios if item.line_number == wanted_line), None)
    if scenario is None:
        raise ValueError(f"Scenario line {wanted_line} is missing from {scenario_path}")
    if scenario.start_id != int(inputs["start"]) or scenario.goal_id != int(inputs["goal"]):
        raise ValueError(f"Campaign endpoints do not match {scenario_path}:{wanted_line}")
    expected_hash = inputs.get("map_sha256")
    if expected_hash and scenario.map.map_sha256 != expected_hash:
        raise ValueError(f"Map hash changed for {scenario.map_path}")
    return scenario


def _cell_mask(cells: set[tuple[int, int]], width: int, height: int, side: int) -> np.ndarray:
    mask = np.zeros((height, width), dtype=bool)
    for cell_side, node in cells:
        if cell_side != side or node < 0 or node >= width * height:
            continue
        mask[node // width, node % width] = True
    return mask


def _draw_overlay(ax: Any, mask: np.ndarray, color: str, alpha: float) -> None:
    if not np.any(mask):
        return
    rgba = np.zeros((*mask.shape, 4), dtype=float)
    rgba[mask] = (*to_rgba(color)[:3], alpha)
    ax.imshow(rgba, origin="upper", interpolation="nearest", zorder=1)


def _draw_scenario(
    scenario: MovingAIScenario,
    record: dict[str, Any],
    *,
    planner: str,
    params: dict[str, object],
    output_path: Path,
    trace_bytes: int,
) -> None:
    problem = DiscreteProblem(
        graph=scenario.map,
        start=scenario.start_id,
        goal=scenario.goal_id,
        params={"max_materialized_nodes": scenario.map.node_count},
    )
    result = plan_discrete(
        problem,
        planner=planner,
        params=params or None,
        trace=TraceOptions(max_bytes=trace_bytes),
    )
    if not result.success or result.path is None or result.trace is None:
        raise RuntimeError(
            f"{planner} did not produce a visualizable route for {scenario.map_path}"
        )

    path_ids = np.asarray(result.path, dtype=np.int64).reshape(-1)
    path_xy = np.column_stack((path_ids % scenario.map.width, path_ids // scenario.map.width))
    reference_cost = float(record["input"]["reference_cost"])
    path_cost = float(result.stats["path_cost"])
    cost_ratio = path_cost / reference_cost if reference_cost else 1.0
    controller = ReplayController(
        result.trace, checkpoint_stride=max(1, len(result.trace.events) + 1), max_cache_bytes=0
    )
    state = controller.seek(controller.length)

    figure_height = 6.7
    map_aspect = scenario.map.width / scenario.map.height
    figure_width = max(7.2, min(9.5, map_aspect * figure_height / 0.7))
    figure, ax = plt.subplots(figsize=(figure_width, figure_height))
    ax.imshow(
        scenario.map.occupancy.astype(np.uint8),
        origin="upper",
        interpolation="nearest",
        cmap=ListedColormap(["#f8fafc", "#344054"]),
        vmin=0,
        vmax=1,
        zorder=0,
    )

    side_ids = sorted({side for side, _ in state.visited | state.frontier})
    bidirectional = planner.startswith("bidirectional_")
    if bidirectional:
        if len(side_ids) < 2:
            side_ids = [1, 2]
        side_labels = ("Start-side", "Goal-side")
        side_colors = (EXPLORED_COLORS[0], EXPLORED_COLORS[1])
        frontier_labels = ("Start frontier", "Goal frontier")
    else:
        side_ids = side_ids[:1] or [0]
        side_labels = ("Expanded",)
        side_colors = (EXPLORED_COLORS[0],)
        frontier_labels = ("Frontier",)

    for index, side in enumerate(side_ids[: len(side_colors)]):
        _draw_overlay(
            ax,
            _cell_mask(state.visited, scenario.map.width, scenario.map.height, side),
            side_colors[index],
            0.56,
        )
        _draw_overlay(
            ax,
            _cell_mask(state.frontier, scenario.map.width, scenario.map.height, side),
            FRONTIER_COLORS[index],
            0.78,
        )

    ax.plot(
        path_xy[:, 0],
        path_xy[:, 1],
        color=PATH_COLOR,
        linewidth=1.35,
        solid_capstyle="round",
        zorder=4,
        rasterized=True,
    )
    start_x, start_y = scenario.map.cell_for(scenario.start_id)
    goal_x, goal_y = scenario.map.cell_for(scenario.goal_id)
    ax.scatter(
        [start_x],
        [start_y],
        s=76,
        c=START_COLOR,
        marker="o",
        edgecolors="black",
        linewidths=0.6,
        zorder=5,
    )
    ax.scatter(
        [goal_x],
        [goal_y],
        s=102,
        c=GOAL_COLOR,
        marker="*",
        edgecolors="black",
        linewidths=0.6,
        zorder=5,
    )

    handles = [
        Patch(facecolor=color, alpha=0.7, label=label)
        for color, label in zip(side_colors, side_labels, strict=True)
    ]
    handles.extend(
        Patch(facecolor=FRONTIER_COLORS[index], alpha=0.8, label=frontier_labels[index])
        for index in range(len(side_ids[: len(side_colors)]))
    )
    handles.extend(
        [
            Line2D([], [], color=PATH_COLOR, linewidth=2, label="Path"),
            Line2D(
                [],
                [],
                marker="o",
                linestyle="",
                color=START_COLOR,
                markeredgecolor="black",
                label="Start",
            ),
            Line2D(
                [],
                [],
                marker="*",
                linestyle="",
                color=GOAL_COLOR,
                markeredgecolor="black",
                label="Goal",
            ),
        ]
    )

    height, width = scenario.map.occupancy.shape
    ax.set_xlim(-0.5, width - 0.5)
    ax.set_ylim(height - 0.5, -0.5)
    ax.set_xticks(sorted({0, (width - 1) // 2, width - 1}))
    ax.set_yticks(sorted({0, (height - 1) // 2, height - 1}))
    ax.set_xlabel("Column")
    ax.set_ylabel("Row")
    ax.grid(False)
    ax.set_aspect("equal", adjustable="box")
    for spine in ax.spines.values():
        spine.set_color("#98a2b3")

    figure.suptitle(
        f"{ALGORITHM_LABELS[planner]} · {FAMILY_LABELS[scenario.family]}",
        x=0.10,
        y=0.97,
        ha="left",
        fontsize=15,
        fontweight="bold",
    )
    ax.set_title(
        f"{scenario.map_path.name} · scenario line {scenario.line_number}",
        loc="left",
        fontsize=10,
        pad=8,
    )
    figure.subplots_adjust(left=0.09, right=0.98, top=0.88, bottom=0.20)
    figure.legend(
        handles=handles,
        loc="lower center",
        bbox_to_anchor=(0.52, 0.075),
        ncol=min(5, len(handles)),
        frameon=False,
        fontsize=8,
    )
    trace_note = "recorded trace truncated" if result.trace.truncated else "complete trace"
    figure.text(
        0.10,
        0.025,
        f"{result.iters:,} expanded · {len(path_ids):,} path cells · cost ratio {cost_ratio:.3f} · {trace_note}",
        fontsize=8,
        color="#475467",
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output_path, dpi=170, bbox_inches="tight", facecolor="white")
    plt.close(figure)
    print(
        f"wrote {output_path} from {scenario.map_path.relative_to(scenario.dataset_root)}:"
        f"{scenario.line_number} ({planner})"
    )


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--campaign", type=Path, default=DEFAULT_CAMPAIGN)
    parser.add_argument("--dataset-root", type=Path, default=DEFAULT_DATASET_ROOT)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument(
        "--trace-bytes",
        type=int,
        default=64 * 1024 * 1024,
        help="Maximum diagnostic trace storage for each rendered query",
    )
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    if args.trace_bytes <= 0:
        raise ValueError("--trace-bytes must be positive")
    rows_by_family = _candidate_rows(args.campaign)
    for family, planner, params, filename in SCENES:
        record = _select_case(rows_by_family, family)
        scenario = _resolve_scenario(record, args.dataset_root)
        _draw_scenario(
            scenario,
            record,
            planner=planner,
            params=params,
            output_path=args.output_dir / filename,
            trace_bytes=args.trace_bytes,
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
