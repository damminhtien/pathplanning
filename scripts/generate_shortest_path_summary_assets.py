#!/usr/bin/env python3
"""Render README benchmark figures from a shortest-path summary.json."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import scienceplots  # noqa: F401  # Register the package's Matplotlib styles.

plt.style.use(["science", "no-latex"])
plt.rcParams.update(
    {
        "font.size": 9,
        "axes.titlesize": 10,
        "axes.labelsize": 9,
        "xtick.labelsize": 8,
        "ytick.labelsize": 8,
        "legend.fontsize": 8,
        "savefig.dpi": 180,
    }
)

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SUMMARY = Path("benchmark-results/pilot/summary.json")
DEFAULT_OUTPUT_DIR = Path("assets/images")

VARIANT_LABELS = {
    "dijkstra": "Dijkstra",
    "bidirectional_dijkstra": "Bidirectional Dijkstra",
    "astar": "A*",
    "bidirectional_astar": "Bidirectional A*",
    "weighted_astar_1.25": "Weighted A* (1.25)",
    "weighted_astar_1.5": "Weighted A* (1.5)",
    "weighted_astar_2": "Weighted A* (2.0)",
    "greedy_best_first": "Greedy best-first",
}
VARIANT_COLORS = {
    "dijkstra": "#0072B2",
    "bidirectional_dijkstra": "#56B4E9",
    "astar": "#009E73",
    "bidirectional_astar": "#005A46",
    "weighted_astar_1.25": "#E69F00",
    "weighted_astar_1.5": "#D55E00",
    "weighted_astar_2": "#CC79A7",
    "greedy_best_first": "#6C71C4",
}


def _resolve(path: Path) -> Path:
    return path if path.is_absolute() else ROOT / path


def _read_summary(path: Path) -> dict[str, Any]:
    summary = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(summary, dict) or not isinstance(summary.get("cohorts"), list):
        raise ValueError(f"{path} is not a shortest-path campaign summary")
    return summary


def _cohort(summary: dict[str, Any], pass_name: str) -> dict[str, Any]:
    matches = [
        row
        for row in summary["cohorts"]
        if row.get("pass") == pass_name
        and row.get("protocol", {}).get("scope") == "public_api"
        and row.get("protocol", {}).get("graph_state") == "reused_graph"
        and row.get("protocol", {}).get("movement_profile") == "land_octile_v1"
    ]
    if len(matches) != 1:
        raise ValueError(
            f"Expected one public_api/reused_graph/land_octile_v1 {pass_name!r} cohort; "
            f"found {len(matches)}"
        )
    return matches[0]


def _variant_rows(cohort: dict[str, Any]) -> list[tuple[str, str, dict[str, Any]]]:
    variants = cohort.get("variants")
    if not isinstance(variants, dict) or not variants:
        raise ValueError(f"The {cohort.get('pass')} cohort contains no variants")
    names = [name for name in VARIANT_LABELS if name in variants]
    names.extend(sorted(name for name in variants if name not in VARIANT_LABELS))
    return [
        (name, VARIANT_LABELS.get(name, name.replace("_", " ")), variants[name]) for name in names
    ]


def _numeric(value: Any, *, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"Expected a numeric value for {field}, got {value!r}")
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"Expected a finite value for {field}, got {value!r}")
    return number


def _metric_stats(variant: dict[str, Any], group: str, metric: str) -> tuple[float, float, int]:
    stats = variant.get(group, {}).get(metric)
    if not isinstance(stats, dict):
        raise ValueError(f"Missing {group}.{metric} in the summary")
    median = _numeric(stats.get("median"), field=f"{group}.{metric}.median")
    p95 = _numeric(stats.get("p95"), field=f"{group}.{metric}.p95")
    count = int(_numeric(stats.get("count"), field=f"{group}.{metric}.count"))
    if median <= 0 or p95 < median or count <= 0:
        raise ValueError(f"Invalid distribution for {group}.{metric}: {stats!r}")
    return median, p95, count


def _save(figure: Any, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path, bbox_inches="tight", facecolor="white")
    plt.close(figure)


def _plot_distribution(
    axis: Any,
    rows: list[tuple[str, str, dict[str, Any]]],
    *,
    group: str,
    metric: str,
    title: str,
    xlabel: str,
    scale: float = 1.0,
    show_labels: bool = True,
) -> None:
    positions = list(range(len(rows)))
    counts = set()
    for position, (name, _label, variant) in zip(positions, rows, strict=True):
        median, p95, count = _metric_stats(variant, group, metric)
        counts.add(count)
        median /= scale
        p95 /= scale
        axis.errorbar(
            median,
            position,
            xerr=[[0.0], [p95 - median]],
            fmt="o",
            markersize=4.5,
            capsize=2.5,
            linewidth=1.1,
            color=VARIANT_COLORS.get(name, "#444444"),
            ecolor=VARIANT_COLORS.get(name, "#444444"),
            zorder=3,
        )
    if len(counts) != 1:
        raise ValueError(f"{group}.{metric} has inconsistent counts across variants: {counts}")
    axis.set_xscale("log")
    axis.set_yticks(positions)
    if show_labels:
        axis.set_yticklabels([label for _name, label, _variant in rows])
    else:
        axis.tick_params(axis="y", labelleft=False)
    axis.set_title(f"{title}, n={counts.pop():,}")
    axis.set_xlabel(xlabel)
    axis.grid(axis="x", which="both", alpha=0.28, linewidth=0.6)
    axis.set_axisbelow(True)


def _plot_latency(cohort: dict[str, Any], path: Path) -> None:
    rows = _variant_rows(cohort)
    figure, axis = plt.subplots(figsize=(10, 5.0), layout="constrained")
    _plot_distribution(
        axis,
        rows,
        group="timing",
        metric="query_medians_s",
        title="Public API latency per query",
        xlabel="Latency (ms, log scale)",
        scale=0.001,
    )
    axis.invert_yaxis()
    figure.suptitle(
        "MovingAI pilot latency\nMedian marker; whisker extends to P95 of per-query medians",
        fontsize=12,
    )
    _save(figure, path)


def _plot_work_quality(cohort: dict[str, Any], path: Path) -> None:
    rows = _variant_rows(cohort)
    figure, axes = plt.subplots(2, 2, figsize=(11, 7.5), sharey=True)
    metrics = (
        (axes[0, 0], "expanded", "Expanded nodes", True),
        (axes[0, 1], "edges_examined", "Edges examined", False),
        (axes[1, 0], "frontier_pushes", "Frontier pushes", True),
    )
    for axis, metric, title, show_labels in metrics:
        _plot_distribution(
            axis,
            rows,
            group="work",
            metric=metric,
            title=title,
            xlabel="Operations per query (log scale)",
            show_labels=show_labels,
        )

    quality_axis = axes[1, 1]
    quality_counts = {
        int(
            _numeric(
                variant.get("quality", {}).get("mean_cost_ratio_denominator"),
                field="quality.mean_cost_ratio_denominator",
            )
        )
        for _name, _label, variant in rows
    }
    if len(quality_counts) != 1:
        raise ValueError(f"Mean path-cost ratios have inconsistent denominators: {quality_counts}")
    quality_axis.set_title(f"Mean path-cost ratio, n={quality_counts.pop():,}")
    quality_axis.set_xlabel("Mean candidate cost / oracle cost (1.0 = optimal)")
    quality_axis.set_yticks(list(range(len(rows))))
    quality_axis.tick_params(axis="y", labelleft=False)
    ratios = [
        _numeric(
            variant.get("quality", {}).get("mean_cost_ratio"),
            field="quality.mean_cost_ratio",
        )
        for _name, _label, variant in rows
    ]
    ratios = [1.0 if math.isclose(value, 1.0, abs_tol=1e-12) else value for value in ratios]
    quality_axis.axvline(1.0, color="#444444", linestyle="--", linewidth=1)
    for position, ((name, _label, _variant), ratio) in enumerate(zip(rows, ratios, strict=True)):
        color = VARIANT_COLORS.get(name, "#444444")
        quality_axis.hlines(position, 1.0, ratio, color=color, linewidth=2.3, alpha=0.8)
        quality_axis.plot(ratio, position, "o", color=color, markersize=4.5, zorder=3)
    quality_axis.set_xlim(left=0.99, right=max(1.02, max(ratios) * 1.04))
    quality_axis.grid(axis="x", alpha=0.28, linewidth=0.6)
    quality_axis.set_axisbelow(True)
    axes[0, 0].invert_yaxis()

    figure.suptitle(
        "MovingAI pilot search work and solution quality\n"
        "Work markers are medians; whiskers extend to P95 across the work cohort",
        fontsize=12,
    )
    figure.tight_layout(rect=(0, 0, 1, 0.91))
    _save(figure, path)


def _plot_memory(work_cohort: dict[str, Any], memory_cohort: dict[str, Any], path: Path) -> None:
    work_rows = _variant_rows(work_cohort)
    memory_rows = _variant_rows(memory_cohort)
    if [name for name, _label, _variant in work_rows] != [
        name for name, _label, _variant in memory_rows
    ]:
        raise ValueError("Work and memory cohorts contain different variants")
    figure, axes = plt.subplots(1, 2, figsize=(11, 5.1), sharey=True)
    _plot_distribution(
        axes[0],
        work_rows,
        group="memory",
        metric="query_workspace_peak_bytes",
        title="Tracked query workspace",
        xlabel="Workspace (MiB, log scale)",
        scale=1024 * 1024,
    )
    _plot_distribution(
        axes[1],
        memory_rows,
        group="memory",
        metric="process_peak_rss_bytes",
        title="Fresh-worker process RSS",
        xlabel="RSS (MiB, log scale)",
        scale=1024 * 1024,
        show_labels=False,
    )
    axes[0].invert_yaxis()
    figure.suptitle(
        "MovingAI pilot memory\n"
        "Separate work and memory passes; RSS includes Python, map loading, and graph setup",
        fontsize=12,
    )
    figure.tight_layout(rect=(0, 0, 1, 0.91))
    _save(figure, path)


def generate_assets(summary_path: Path, output_dir: Path) -> list[Path]:
    summary = _read_summary(summary_path)
    latency = _cohort(summary, "latency")
    work = _cohort(summary, "work")
    memory = _cohort(summary, "memory")
    paths = [
        output_dir / "movingai-pilot-latency.png",
        output_dir / "movingai-pilot-work-quality.png",
        output_dir / "movingai-pilot-memory.png",
    ]
    _plot_latency(latency, paths[0])
    _plot_work_quality(work, paths[1])
    _plot_memory(work, memory, paths[2])
    return paths


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--summary", type=Path, default=DEFAULT_SUMMARY)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    args = parser.parse_args()
    summary_path = _resolve(args.summary)
    output_dir = _resolve(args.output_dir)
    for path in generate_assets(summary_path, output_dir):
        print(path.relative_to(ROOT) if path.is_relative_to(ROOT) else path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
