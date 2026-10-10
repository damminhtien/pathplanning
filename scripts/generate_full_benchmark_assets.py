#!/usr/bin/env python3
"""Render README figures from the latest complete benchmark cohorts."""

from __future__ import annotations

import argparse
from collections import defaultdict
import json
import math
from pathlib import Path
import statistics
from typing import Any

import matplotlib

matplotlib.use("Agg")

from matplotlib.patches import Patch
import matplotlib.pyplot as plt
import scienceplots  # noqa: F401  # Register the package's Matplotlib styles.

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CAMPAIGN_ROOT = Path("benchmark-results/refresh_20261010")
DEFAULT_OUTPUT_DIR = Path("assets/images")

VARIANT_LABELS = {
    "abit_star": "ABIT*",
    "ait_star": "AIT*",
    "anytime_astar": "Anytime A*",
    "astar": "A*",
    "astar_halpha_0": "A* (α=0)",
    "astar_halpha_0.25": "A* (α=0.25)",
    "astar_halpha_0.5": "A* (α=0.5)",
    "astar_halpha_0.75": "A* (α=0.75)",
    "astar_halpha_1": "A* (α=1)",
    "bfs": "BFS",
    "bidirectional_astar": "Bidirectional A*",
    "bidirectional_dijkstra": "Bidirectional Dijkstra",
    "bit_star": "BIT*",
    "breadth_first_search": "BFS",
    "depth_first_search": "DFS",
    "dfs": "DFS",
    "dijkstra": "Dijkstra",
    "dstar_lite": "D* Lite",
    "eirm_star": "EIRM*",
    "eit_star": "EIT*",
    "fcit_star": "FCIT*",
    "fmt_star": "FMT*",
    "greedy_best_first": "Greedy best-first",
    "informed_rrt_star": "Informed RRT*",
    "jps": "JPS",
    "lazy_prm": "Lazy PRM",
    "lazy_theta_star": "Lazy Theta*",
    "prm_star": "PRM*",
    "reexp_astar": "ReExp A*",
    "rit_star": "RIT*",
    "rrt": "RRT",
    "rrt_connect": "RRT-Connect",
    "rrt_star": "RRT*",
    "theta_star": "Theta*",
    "weighted_astar": "Weighted A*",
    "weighted_astar_1.25": "Weighted A* (1.25)",
    "weighted_astar_1.5": "Weighted A* (1.5)",
    "weighted_astar_2": "Weighted A* (2.0)",
}

COHORT_LABELS = {
    "dimacs_distance_directed_weighted_graph": "DIMACS distance",
    "dimacs_source_weight_directed_weighted_graph": "DIMACS source-weight",
    "dimacs_travel_time_directed_weighted_graph": "DIMACS travel time",
    "monash_industrial-plants_strict_26_euclidean": "Monash voxel",
    "movingai_warframe_strict_26_euclidean": "MovingAI voxel",
    "barn_point_xy_derived": "BARN point-robot XY",
}

COHORT_COLORS = {
    "movingai": "#246A8D",
    "scaling": "#27856A",
    "dimacs": "#AE6547",
    "movingai_voxel": "#7866A8",
    "monash_voxel": "#9A6AA8",
    "barn": "#C77B24",
    "grid": "#477C9B",
}

STATUS_LABELS = {
    "no_solution_found": "No solution found",
    "proved_unreachable": "Unreachable confirmed",
    "resource_limited": "Resource limited",
    "valid_any_angle_path": "Valid any-angle path",
    "valid_optimal": "Valid and optimal",
    "valid_path": "Valid path",
    "valid_suboptimal": "Valid, suboptimal",
    "error": "Runner error",
    "unclassified": "Unclassified",
}
STATUS_COLORS = {
    "valid_optimal": "#248C79",
    "valid_path": "#51A995",
    "valid_any_angle_path": "#6BAFC0",
    "valid_suboptimal": "#D5A13B",
    "no_solution_found": "#D87961",
    "proved_unreachable": "#7667A9",
    "resource_limited": "#7F8B98",
    "error": "#B94445",
    "unclassified": "#A8AFB8",
}

EXPECTED_FORMAT_COHORTS = tuple(COHORT_LABELS)


def _resolve(path: Path) -> Path:
    return path if path.is_absolute() else ROOT / path


def _read_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"Expected a JSON object in {path}")
    return value


def _read_jsonl(path: Path) -> list[dict[str, Any]]:
    rows = []
    with path.open(encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, start=1):
            try:
                row = json.loads(line)
            except json.JSONDecodeError as exc:
                raise ValueError(f"Invalid JSON at {path}:{line_number}: {exc}") from exc
            if not isinstance(row, dict):
                raise ValueError(f"Expected an object at {path}:{line_number}")
            rows.append(row)
    return rows


def _summary(path: Path) -> dict[str, Any]:
    value = _read_json(path)
    if not isinstance(value.get("cohorts"), list):
        raise ValueError(f"{path} is not an analyzed shortest-path campaign")
    return value


def _summary_cohort(summary: dict[str, Any], pass_name: str) -> dict[str, Any]:
    matches = [cohort for cohort in summary["cohorts"] if cohort.get("pass") == pass_name]
    if len(matches) != 1:
        raise ValueError(f"Expected one {pass_name!r} pass, found {len(matches)}")
    cohort = matches[0]
    protocol = cohort.get("protocol", {})
    if protocol.get("scope") != "public_api" or protocol.get("graph_state") != "reused_graph":
        raise ValueError(f"{pass_name!r} pass is not public_api/reused_graph")
    if protocol.get("movement_profile") != "land_octile_v1":
        raise ValueError(f"{pass_name!r} pass is not the land_octile_v1 profile")
    if not isinstance(cohort.get("variants"), dict) or not cohort["variants"]:
        raise ValueError(f"{pass_name!r} pass has no variants")
    return cohort


def _number(value: Any, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"Expected a number for {field}, received {value!r}")
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"Expected a finite number for {field}, received {value!r}")
    return number


def _display_variant(name: str, parameters: dict[str, Any] | None = None) -> str:
    label = VARIANT_LABELS.get(name, name.replace("_", " "))
    if name == "weighted_astar" and parameters:
        weight = parameters.get("weight") or parameters.get("heuristic_weight")
        if weight is not None:
            label = f"Weighted A* ({weight:g})"
    return label


def _ordered_variants(variants: dict[str, Any]) -> list[tuple[str, str, dict[str, Any]]]:
    rows = [
        (name, _display_variant(name), value)
        for name, value in variants.items()
        if isinstance(value, dict)
    ]
    return sorted(rows, key=lambda row: row[1].casefold())


def _style() -> None:
    plt.style.use(["science", "no-latex"])
    plt.rcParams.update(
        {
            "font.size": 8,
            "axes.titlesize": 9,
            "axes.labelsize": 8,
            "xtick.labelsize": 7,
            "ytick.labelsize": 7,
            "legend.fontsize": 7,
            "axes.titleweight": "bold",
            "axes.edgecolor": "#9AA7B2",
            "axes.labelcolor": "#273746",
            "text.color": "#182B3A",
            "xtick.color": "#425466",
            "ytick.color": "#273746",
            "grid.color": "#D5DDE4",
            "grid.linewidth": 0.55,
            "savefig.dpi": 180,
            "svg.fonttype": "none",
            "svg.hashsalt": "pathplanning-benchmark-v1",
        }
    )


def _save(figure: Any, path: Path) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path, bbox_inches="tight", facecolor="white")
    svg_path = path.with_suffix(".svg")
    figure.savefig(svg_path, bbox_inches="tight", facecolor="white")
    svg_lines = svg_path.read_text(encoding="utf-8").splitlines()
    svg_path.write_text(
        "\n".join(line.rstrip() for line in svg_lines) + "\n",
        encoding="utf-8",
    )
    plt.close(figure)
    return path


def _draw_latency_rows(
    axis: Any,
    rows: list[tuple[str, float, float, int]],
    *,
    title: str,
    color: str,
) -> None:
    if not rows:
        raise ValueError(f"{title} has no usable latency observations")
    positions = list(range(len(rows)))
    for position, (label, median_s, p95_s, _count) in zip(positions, rows, strict=True):
        median_ms = median_s * 1000
        p95_ms = p95_s * 1000
        axis.errorbar(
            median_ms,
            position,
            xerr=[[0], [max(0, p95_ms - median_ms)]],
            fmt="o",
            markersize=4.1,
            capsize=2.2,
            linewidth=1.15,
            color=color,
            ecolor=color,
            zorder=3,
        )
    axis.set_xscale("log")
    axis.set_yticks(positions)
    axis.set_yticklabels([row[0] for row in rows])
    axis.invert_yaxis()
    axis.set_title(title, loc="left", pad=6)
    axis.set_xlabel("Public API latency (ms, log scale)")
    axis.grid(axis="x", which="both", alpha=0.55)
    axis.grid(axis="y", visible=False)
    axis.set_axisbelow(True)


def _summary_latency_rows(cohort: dict[str, Any]) -> list[tuple[str, float, float, int]]:
    rows = []
    for name, label, variant in _ordered_variants(cohort["variants"]):
        stats = variant.get("timing", {}).get("query_medians_s")
        if not isinstance(stats, dict):
            raise ValueError(f"Missing query latency for {name}")
        median = _number(stats.get("median"), f"{name} latency median")
        p95 = _number(stats.get("p95"), f"{name} latency p95")
        count = int(_number(stats.get("count"), f"{name} latency count"))
        if median <= 0 or p95 < median or count <= 0:
            raise ValueError(f"Invalid latency summary for {name}: {stats}")
        rows.append((label, median, p95, count))
    return rows


def _raw_latency_rows(
    rows: list[dict[str, Any]],
    *,
    cohort_name: str | None = None,
    source: str | None = None,
    planner_field: str = "planner",
    parameters_field: str = "parameters",
) -> tuple[list[tuple[str, float, float, int]], int]:
    selected = [
        row
        for row in rows
        if (cohort_name is None or row.get("cohort") == cohort_name)
        and (source is None or row.get("source") == source)
    ]
    grouped: dict[tuple[str, str], list[float]] = defaultdict(list)
    labels: dict[tuple[str, str], str] = {}
    workload_ids = set()
    for row in selected:
        if row.get("error"):
            continue
        timing = row.get("timing") or {}
        value = timing.get("public_api_s")
        if value is None:
            continue
        seconds = _number(value, f"{row.get('planner')} public_api_s")
        if seconds <= 0:
            continue
        planner = str(row.get(planner_field, "unknown"))
        parameters = row.get(parameters_field) or {}
        parameter_key = json.dumps(parameters, sort_keys=True, separators=(",", ":"))
        key = planner, parameter_key
        grouped[key].append(seconds)
        labels[key] = _display_variant(planner, parameters)
        workload_id = row.get("query_id") or row.get("workload_id") or row.get("dataset_id")
        if workload_id:
            workload_ids.add(str(workload_id))
    aggregate = [
        (labels[key], statistics.median(values), _percentile(values, 95), len(values))
        for key, values in grouped.items()
    ]
    return sorted(aggregate, key=lambda row: row[0].casefold()), len(workload_ids)


def _percentile(values: list[float], percentile: float) -> float:
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    offset = (len(ordered) - 1) * percentile / 100
    lower = math.floor(offset)
    upper = math.ceil(offset)
    fraction = offset - lower
    return ordered[lower] * (1 - fraction) + ordered[upper] * fraction


def generate_latency(
    movingai: dict[str, Any],
    scaling: dict[str, Any],
    format_rows: list[dict[str, Any]],
    grid_rows: list[dict[str, Any]],
    output_dir: Path,
) -> list[Path]:
    movingai_latency = _summary_cohort(movingai, "latency")
    scaling_latency = _summary_cohort(scaling, "latency")
    repeated_panels: list[tuple[str, list[tuple[str, float, float, int]], str]] = [
        (
            "MovingAI land · 5 repeats · n=788 queries",
            _summary_latency_rows(movingai_latency),
            COHORT_COLORS["movingai"],
        ),
        (
            "Scaling grids · 5 repeats · n=386 queries",
            _summary_latency_rows(scaling_latency),
            COHORT_COLORS["scaling"],
        ),
    ]

    single_panels = [
        (
            "dimacs_distance_directed_weighted_graph",
            "DIMACS distance · 6 graphs",
            COHORT_COLORS["dimacs"],
        ),
        (
            "dimacs_travel_time_directed_weighted_graph",
            "DIMACS travel time · 6 graphs",
            COHORT_COLORS["dimacs"],
        ),
        (
            "dimacs_source_weight_directed_weighted_graph",
            "DIMACS source weight · 1 graph",
            COHORT_COLORS["dimacs"],
        ),
        (
            "movingai_warframe_strict_26_euclidean",
            "MovingAI voxel · 1 map",
            COHORT_COLORS["movingai_voxel"],
        ),
        (
            "monash_industrial-plants_strict_26_euclidean",
            "Monash voxel · 1 map",
            COHORT_COLORS["monash_voxel"],
        ),
        ("barn_point_xy_derived", "BARN point-robot XY · 300 worlds", COHORT_COLORS["barn"]),
    ]
    format_panels: list[tuple[str, list[tuple[str, float, float, int]], str]] = []
    specialist_panels: list[tuple[str, list[tuple[str, float, float, int]], str]] = []
    present = {row.get("cohort") for row in format_rows}
    missing = set(EXPECTED_FORMAT_COHORTS) - present
    if missing:
        raise ValueError(f"Format campaign is missing required cohorts: {sorted(missing)}")
    for cohort_name, title, color in single_panels:
        rows, count = _raw_latency_rows(format_rows, cohort_name=cohort_name)
        format_panels.append(
            (
                f"{title} · one call · n={count}",
                rows,
                color,
            )
        )

    grid_group = {row.get("cohort") for row in grid_rows}
    expected_grid_groups = {"movingai_land_octile", "movingai_land_any_angle"}
    if grid_group != expected_grid_groups:
        raise ValueError(f"Unexpected grid-specialist cohorts: {sorted(grid_group)}")
    for cohort_name, label in (
        ("movingai_land_octile", "octile grid"),
        ("movingai_land_any_angle", "any-angle grid"),
    ):
        rows, count = _raw_latency_rows(grid_rows, cohort_name=cohort_name)
        specialist_panels.append(
            (
                f"MovingAI {label} specialists · one call · n={count} pairs",
                rows,
                COHORT_COLORS["grid"],
            )
        )

    figure, axes = plt.subplots(1, 2, figsize=(15, 8.2), layout="constrained")
    for axis, (title, rows, color) in zip(axes, repeated_panels, strict=True):
        _draw_latency_rows(axis, rows, title=title, color=color)
    figure.suptitle(
        "Repeated-query latency for compatible 2D grid cohorts\n"
        "Point = median of five calls per query; whisker = P95 across query medians",
        fontsize=12,
    )
    paths = [_save(figure, output_dir / "benchmark-full-latency.png")]

    figure, axes = plt.subplots(1, 2, figsize=(13, 2.6), layout="constrained")
    for axis, (title, rows, color) in zip(axes, specialist_panels, strict=True):
        _draw_latency_rows(axis, rows, title=title, color=color)
    figure.suptitle(
        "One-call latency for MovingAI grid specialists\n"
        "Point = median; one call per map/query pair; whisker = P95 across pairs",
        fontsize=12,
    )
    paths.append(_save(figure, output_dir / "benchmark-full-latency-specialists.png"))

    figure, axes = plt.subplots(2, 3, figsize=(17, 10.5), layout="constrained")
    flat_axes = list(axes.flat)
    for axis, (title, rows, color) in zip(flat_axes, format_panels, strict=True):
        _draw_latency_rows(axis, rows, title=title, color=color)
    figure.suptitle(
        "Latency across separate source-format cohorts\n"
        "One call per instance; point = cohort median, whisker = P95. n=1 panels are single cases.",
        fontsize=12,
    )
    paths.append(_save(figure, output_dir / "benchmark-full-latency-formats.png"))
    return paths


def _plot_work_metric(
    axis: Any,
    rows: list[tuple[str, str, dict[str, Any]]],
    *,
    metric: str,
    title: str,
    color: str,
    show_labels: bool,
) -> None:
    values = []
    counts = set()
    for _name, label, variant in rows:
        stats = variant.get("work", {}).get(metric)
        if not isinstance(stats, dict):
            raise ValueError(f"Missing work.{metric} for {label}")
        median = _number(stats.get("median"), f"{label} {metric} median")
        p95 = _number(stats.get("p95"), f"{label} {metric} p95")
        count = int(_number(stats.get("count"), f"{label} {metric} count"))
        if median < 0 or p95 < median or count <= 0:
            raise ValueError(f"Invalid {metric} summary for {label}: {stats}")
        values.append((median, p95))
        counts.add(count)
    if len(counts) != 1:
        raise ValueError(f"{metric} denominator differs among variants: {counts}")
    for position, (median, p95) in enumerate(values):
        axis.errorbar(
            max(median, 0.01),
            position,
            xerr=[[0], [max(0, p95 - median)]],
            fmt="o",
            markersize=3.7,
            capsize=2.0,
            linewidth=1.05,
            color=color,
            ecolor=color,
            zorder=3,
        )
    axis.set_xscale("log")
    axis.set_title(f"{title} · n={counts.pop():,}", loc="left", pad=5)
    axis.set_xlabel("Operations per task (log scale)")
    axis.set_yticks(range(len(rows)))
    if show_labels:
        axis.set_yticklabels([row[1] for row in rows])
    else:
        axis.tick_params(axis="y", labelleft=False)
    axis.grid(axis="x", which="both", alpha=0.55)
    axis.grid(axis="y", visible=False)
    axis.set_axisbelow(True)


def _plot_cost_ratio(
    axis: Any,
    rows: list[tuple[str, str, dict[str, Any]]],
    *,
    title: str,
    color: str,
    show_labels: bool,
) -> None:
    ratios = []
    counts = set()
    for _name, label, variant in rows:
        quality = variant.get("quality", {})
        ratio = _number(quality.get("mean_cost_ratio"), f"{label} mean cost ratio")
        count = int(
            _number(quality.get("mean_cost_ratio_denominator"), f"{label} cost ratio count")
        )
        if ratio < 1 - 1e-10 or count <= 0:
            raise ValueError(f"Invalid cost ratio for {label}: {ratio}, n={count}")
        ratios.append(1.0 if math.isclose(ratio, 1.0, abs_tol=1e-10) else ratio)
        counts.add(count)
    if len(counts) != 1:
        raise ValueError(f"Cost-ratio denominator differs among variants: {counts}")
    axis.axvline(1.0, color="#73818C", linestyle=(0, (3, 2)), linewidth=1)
    for position, ratio in enumerate(ratios):
        axis.hlines(position, 1.0, ratio, color=color, linewidth=2.3, alpha=0.65)
        axis.plot(ratio, position, "o", color=color, markersize=4.2, zorder=3)
        axis.annotate(
            f"{ratio:.3g}×",
            (ratio, position),
            xytext=(5, 0),
            textcoords="offset points",
            fontsize=6,
            va="center",
        )
    axis.set_title(f"{title} · n={counts.pop():,}", loc="left", pad=5)
    axis.set_xlabel("Mean path cost / oracle cost · 1.0 = optimal")
    axis.set_yticks(range(len(rows)))
    if show_labels:
        axis.set_yticklabels([row[1] for row in rows])
    else:
        axis.tick_params(axis="y", labelleft=False)
    axis.set_xscale("log")
    axis.set_xlim(left=0.99, right=max(ratios) * 1.7)
    axis.grid(axis="x", which="both", alpha=0.55)
    axis.grid(axis="y", visible=False)
    axis.set_axisbelow(True)


def generate_work_quality(
    movingai: dict[str, Any], scaling: dict[str, Any], output_dir: Path
) -> Path:
    movingai_work = _summary_cohort(movingai, "work")
    scaling_work = _summary_cohort(scaling, "work")
    groups = [
        ("MovingAI land", _ordered_variants(movingai_work["variants"]), COHORT_COLORS["movingai"]),
        ("Scaling grids", _ordered_variants(scaling_work["variants"]), COHORT_COLORS["scaling"]),
    ]
    figure, axes = plt.subplots(2, 4, figsize=(16.5, 8.6), layout="constrained")
    metrics = (
        ("expanded", "Expanded nodes"),
        ("edges_examined", "Edges examined"),
        ("frontier_pushes", "Frontier pushes"),
    )
    for row_index, (cohort_name, variants, color) in enumerate(groups):
        for column, (metric, title) in enumerate(metrics):
            _plot_work_metric(
                axes[row_index, column],
                variants,
                metric=metric,
                title=title,
                color=color,
                show_labels=column == 0,
            )
        _plot_cost_ratio(
            axes[row_index, 3],
            variants,
            title=f"{cohort_name} mean cost ratio",
            color=color,
            show_labels=False,
        )
        axes[row_index, 0].invert_yaxis()
        axes[row_index, 3].invert_yaxis()
        axes[row_index, 0].text(
            -0.40,
            1.04,
            cohort_name,
            transform=axes[row_index, 0].transAxes,
            ha="left",
            va="bottom",
            fontsize=10,
            fontweight="bold",
            color=color,
        )
    figure.suptitle(
        "Search work and solution quality for 2D grid cohorts\n"
        "Work markers are medians with P95 whiskers across tasks; cost ratio is the cohort mean",
        fontsize=13,
        fontweight="bold",
        color="#203746",
    )
    return _save(figure, output_dir / "benchmark-full-work-quality.png")


def _status(row: dict[str, Any], *, status_field: str = "status") -> str:
    outcome = row.get("outcome") or {}
    status = outcome.get(status_field)
    if status:
        return str(status)
    if row.get("error") or row.get("error_stage"):
        return "error"
    return "unclassified"


def _variant_key(row: dict[str, Any]) -> tuple[str, str]:
    planner = str(row.get("planner") or row.get("variant_name") or "unknown")
    parameters = row.get("parameters") or row.get("variant_parameters") or {}
    return planner, json.dumps(parameters, sort_keys=True, separators=(",", ":"))


def _variant_label_from_row(row: dict[str, Any]) -> str:
    planner, parameters_json = _variant_key(row)
    return _display_variant(planner, json.loads(parameters_json))


def _outcome_rows(
    rows: list[dict[str, Any]],
    *,
    cohort_name: str | None = None,
    status_field: str = "status",
) -> tuple[list[tuple[str, dict[str, int], int]], int]:
    selected = [
        row
        for row in rows
        if (cohort_name is None or row.get("cohort") == cohort_name)
        and (not row.get("phase") or row.get("phase") == "measured")
    ]
    grouped: dict[tuple[str, str], dict[str, int]] = defaultdict(lambda: defaultdict(int))
    counts: dict[tuple[str, str], int] = defaultdict(int)
    ids = set()
    for row in selected:
        key = _variant_key(row)
        grouped[key][_status(row, status_field=status_field)] += 1
        counts[key] += 1
        workload_id = row.get("query_id") or row.get("workload_id") or row.get("dataset_id")
        if workload_id:
            ids.add(str(workload_id))
    result = [
        (_display_variant(key[0], json.loads(key[1])), dict(statuses), counts[key])
        for key, statuses in grouped.items()
    ]
    return sorted(result, key=lambda row: row[0].casefold()), len(ids)


def _draw_outcome_panel(
    axis: Any,
    rows: list[tuple[str, dict[str, int], int]],
    *,
    title: str,
    workload_count: int,
) -> set[str]:
    if not rows:
        raise ValueError(f"{title} has no outcomes")
    statuses = sorted(
        {status for _label, counts, _total in rows for status in counts},
        key=lambda status: (
            0 if status in ("valid_optimal", "proved_unreachable") else 1,
            status,
        ),
    )
    positions = list(range(len(rows)))
    left = [0.0] * len(rows)
    for status in statuses:
        widths = []
        for _label, counts, total in rows:
            denominator = max(1, total)
            widths.append(counts.get(status, 0) * 100 / denominator)
        axis.barh(
            positions,
            widths,
            left=left,
            height=0.67,
            color=STATUS_COLORS.get(status, STATUS_COLORS["unclassified"]),
            edgecolor="white",
            linewidth=0.45,
            label=STATUS_LABELS.get(status, status.replace("_", " ")),
        )
        left = [offset + width for offset, width in zip(left, widths, strict=True)]
    axis.set_yticks(positions)
    axis.set_yticklabels([row[0] for row in rows])
    axis.invert_yaxis()
    axis.set_xlim(0, 100)
    axis.set_xticks([0, 25, 50, 75, 100])
    axis.set_xlabel("Outcome share (%)")
    axis.set_title(f"{title} · n={workload_count} per variant", loc="left", pad=5)
    axis.grid(axis="x", alpha=0.55)
    axis.grid(axis="y", visible=False)
    axis.set_axisbelow(True)
    return set(statuses)


def _render_outcome_panels(
    panels: list[tuple[list[tuple[str, dict[str, int], int]], str, int]],
    *,
    output_path: Path,
    title: str,
    subtitle: str,
    columns: int,
) -> Path:
    rows = math.ceil(len(panels) / columns)
    figure, axes = plt.subplots(
        rows,
        columns,
        figsize=(6.4 * columns, 4.2 * rows),
        layout="constrained",
        squeeze=False,
    )
    flat_axes = list(axes.flat)
    statuses_seen: set[str] = set()
    for axis, (cohort_rows, panel_title, workload_count) in zip(flat_axes, panels):
        statuses_seen.update(
            _draw_outcome_panel(
                axis,
                cohort_rows,
                title=panel_title,
                workload_count=workload_count,
            )
        )
    for axis in flat_axes[len(panels) :]:
        axis.set_visible(False)
    legend_handles = [
        Patch(
            facecolor=STATUS_COLORS.get(status, STATUS_COLORS["unclassified"]),
            label=STATUS_LABELS.get(status, status.replace("_", " ")),
        )
        for status in sorted(statuses_seen)
    ]
    figure.legend(
        handles=legend_handles,
        loc="outside lower center",
        ncol=min(5, len(legend_handles)),
        frameon=False,
    )
    figure.suptitle(f"{title}\n{subtitle}", fontsize=12)
    return _save(figure, output_path)


def generate_outcomes(
    format_rows: list[dict[str, Any]],
    movingai_rows: list[dict[str, Any]],
    scaling_rows: list[dict[str, Any]],
    grid_rows: list[dict[str, Any]],
    unreachable_rows: list[dict[str, Any]],
    output_dir: Path,
) -> list[Path]:
    format_groups = [
        ("dimacs_distance_directed_weighted_graph", "DIMACS · distance-weighted directed", 13, 6),
        ("dimacs_travel_time_directed_weighted_graph", "DIMACS · travel-time directed", 13, 6),
        ("dimacs_source_weight_directed_weighted_graph", "DIMACS · source-weight directed", 13, 1),
        ("movingai_warframe_strict_26_euclidean", "MovingAI · voxel / strict 26-neighbor", 13, 1),
        ("monash_industrial-plants_strict_26_euclidean", "Monash · voxel / strict 26-neighbor", 13, 1),
        ("barn_point_xy_derived", "BARN · static point-robot XY", 14, 300),
    ]
    format_panels: list[tuple[list[tuple[str, dict[str, int], int]], str, int]] = []
    for cohort_name, title, expected_variants, expected_workloads in format_groups:
        rows, count = _outcome_rows(format_rows, cohort_name=cohort_name)
        if not rows:
            raise ValueError(f"Format campaign is missing outcomes for {cohort_name}")
        if len(rows) != expected_variants or count != expected_workloads or any(
            total != expected_workloads for _, _, total in rows
        ):
            raise ValueError(f"{cohort_name} does not cover the complete variant/workload set")
        format_panels.append((rows, title, count))

    grid_panels: list[tuple[list[tuple[str, dict[str, int], int]], str, int]] = []
    for source_rows, cohort_name, title, expected_variants, expected_workloads in (
        (movingai_rows, "work", "MovingAI land · octile", 12, 3363),
        (scaling_rows, "work", "Scaling grids · octile", 17, 1600),
    ):
        rows, count = _outcome_rows(
            source_rows,
            cohort_name=cohort_name,
            status_field="execution_status",
        )
        if (
            len(rows) != expected_variants
            or count != expected_workloads
            or any(total != expected_workloads for _, _, total in rows)
        ):
            raise ValueError(f"{title} does not cover all variants on every workload")
        grid_panels.append((rows, title, count))

    for cohort_name, expected_variants in (
        ("movingai_land_octile", 2),
        ("movingai_land_any_angle", 2),
    ):
        grid_rows_summary, grid_count = _outcome_rows(grid_rows, cohort_name=cohort_name)
        if len(grid_rows_summary) != expected_variants or grid_count != 788 or any(
            total != 788 for _, _, total in grid_rows_summary
        ):
            raise ValueError(f"Grid-specialist campaign has no outcomes for {cohort_name}")

    unreachable_summary, unreachable_count = _outcome_rows(
        unreachable_rows,
        status_field="execution_status",
    )
    if not unreachable_summary:
        raise ValueError("Unreachable campaign has no outcomes")
    if (
        len(unreachable_summary) != 12
        or unreachable_count != 2160
        or unreachable_count * 12 != len(unreachable_rows)
    ):
        raise ValueError("Unreachable campaign must cover 12 variants on every workload")
    if any(
        outcomes != {"proved_unreachable": total} or total != unreachable_count
        for _label, outcomes, total in unreachable_summary
    ):
        raise ValueError("Unreachable campaign must prove every workload unreachable")
    grid_path = _render_outcome_panels(
        grid_panels,
        output_path=output_dir / "benchmark-full-outcomes-grids.png",
        title="Outcome mix for repeated 2D octile cohorts",
        subtitle=(
            "Shares use each cohort's own workloads. Grid-specialist results remain separate; "
            f"12 planners proved {unreachable_count:,}/{unreachable_count:,} unreachable queries."
        ),
        columns=2,
    )
    format_path = _render_outcome_panels(
        format_panels,
        output_path=output_dir / "benchmark-full-outcomes-formats.png",
        title="Outcome coverage across source-format cohorts",
        subtitle=(
            "Percentages use each cohort's own workload count; source-only n=1 panels are descriptive. "
            "A BARN no-solution result is separate from path validity."
        ),
        columns=3,
    )
    return [grid_path, format_path]


def _plot_memory_rows(
    axis: Any,
    rows: list[tuple[str, str, dict[str, Any]]],
    *,
    group: str,
    metric: str,
    title: str,
    show_labels: bool,
) -> None:
    points = []
    counts = set()
    for _name, label, variant in rows:
        stats = variant.get(group, {}).get(metric)
        if not isinstance(stats, dict):
            raise ValueError(f"Missing {group}.{metric} for {label}")
        median = _number(stats.get("median"), f"{label} {metric} median")
        p95 = _number(stats.get("p95"), f"{label} {metric} p95")
        count = int(_number(stats.get("count"), f"{label} {metric} count"))
        if median <= 0 or p95 < median or count <= 0:
            raise ValueError(f"Invalid {metric} distribution for {label}: {stats}")
        points.append((median / (1024**2), p95 / (1024**2)))
        counts.add(count)
    if len(counts) != 1:
        raise ValueError(f"{metric} denominator differs among variants: {counts}")
    for position, (median, p95) in enumerate(points):
        axis.errorbar(
            median,
            position,
            xerr=[[0], [p95 - median]],
            fmt="o",
            markersize=3.8,
            capsize=2.0,
            linewidth=1.05,
            color="#BD704E" if "rss" in metric else "#357B89",
            ecolor="#BD704E" if "rss" in metric else "#357B89",
            zorder=3,
        )
    axis.set_xscale("log")
    axis.set_xlabel("MiB (log scale)")
    axis.set_title(f"{title} · n={counts.pop():,}", loc="left", pad=5)
    axis.set_yticks(range(len(rows)))
    if show_labels:
        axis.set_yticklabels([row[1] for row in rows])
    else:
        axis.tick_params(axis="y", labelleft=False)
    axis.grid(axis="x", which="both", alpha=0.55)
    axis.grid(axis="y", visible=False)
    axis.set_axisbelow(True)


def generate_memory(movingai: dict[str, Any], scaling: dict[str, Any], output_dir: Path) -> Path:
    figure, axes = plt.subplots(2, 2, figsize=(15.5, 8.7), layout="constrained")
    for row_index, (name, summary) in enumerate(
        (("MovingAI land", movingai), ("Scaling grids", scaling))
    ):
        work = _summary_cohort(summary, "work")
        memory = _summary_cohort(summary, "memory")
        work_rows = _ordered_variants(work["variants"])
        memory_rows = _ordered_variants(memory["variants"])
        if [item[0] for item in work_rows] != [item[0] for item in memory_rows]:
            raise ValueError(f"{name} work and memory passes use different variants")
        _plot_memory_rows(
            axes[row_index, 0],
            work_rows,
            group="memory",
            metric="query_workspace_peak_bytes",
            title=f"{name} · tracked query workspace · work pass",
            show_labels=True,
        )
        _plot_memory_rows(
            axes[row_index, 1],
            memory_rows,
            group="memory",
            metric="process_peak_rss_bytes",
            title=f"{name} · fresh-worker process RSS · memory pass",
            show_labels=False,
        )
        axes[row_index, 0].invert_yaxis()
        axes[row_index, 1].invert_yaxis()
    figure.suptitle(
        "Memory measurements for the 2D grid cohorts\n"
        "Point = median; whisker = P95. RSS includes Python, map loading, and graph setup",
        fontsize=13,
        fontweight="bold",
        color="#203746",
    )
    return _save(figure, output_dir / "benchmark-full-memory.png")


def generate_assets(campaign_root: Path, output_dir: Path) -> list[Path]:
    movingai = _summary(campaign_root / "movingai_land" / "summary.json")
    scaling = _summary(campaign_root / "scaling" / "summary.json")
    if (movingai.get("reference_workload_count"), movingai.get("raw_run_count")) != (
        3363,
        106908,
    ):
        raise ValueError("MovingAI land campaign is incomplete")
    if (scaling.get("reference_workload_count"), scaling.get("raw_run_count")) != (
        1600,
        76160,
    ):
        raise ValueError("Scaling campaign is incomplete")
    format_root = campaign_root / "format_cohorts"
    format_summary = _read_json(format_root / "summary.json")
    if format_summary.get("input_errors") != 0 or format_summary.get("measurements") != 4395:
        raise ValueError("Expected the complete 4,395-measurement DIMACS/voxel/BARN run")
    format_rows = _read_jsonl(format_root / "runs.jsonl")
    if len(format_rows) != 4395:
        raise ValueError("Format-cohort campaign has an incomplete JSONL log")
    movingai_rows = _read_jsonl(campaign_root / "movingai_land" / "runs.jsonl")
    scaling_rows = _read_jsonl(campaign_root / "scaling" / "runs.jsonl")
    grid_rows = _read_jsonl(campaign_root / "grid_specialists" / "runs.jsonl")
    if len(grid_rows) != 3152:
        raise ValueError("Grid-specialist campaign must contain 3,152 observations")
    unreachable_rows = _read_jsonl(campaign_root / "movingai_unreachable_full" / "runs.jsonl")
    _style()
    return [
        *generate_latency(movingai, scaling, format_rows, grid_rows, output_dir),
        generate_work_quality(movingai, scaling, output_dir),
        *generate_outcomes(
            format_rows,
            movingai_rows,
            scaling_rows,
            grid_rows,
            unreachable_rows,
            output_dir,
        ),
        generate_memory(movingai, scaling, output_dir),
    ]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--campaign-root", type=Path, default=DEFAULT_CAMPAIGN_ROOT)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    args = parser.parse_args()
    campaign_root = _resolve(args.campaign_root)
    output_dir = _resolve(args.output_dir)
    for path in generate_assets(campaign_root, output_dir):
        print(path.relative_to(ROOT) if path.is_relative_to(ROOT) else path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
