"""Coverage-first summaries and planner-independent workload strata."""

from __future__ import annotations

from collections import Counter, defaultdict
import hashlib
import math
from pathlib import Path
from typing import Any

from scripts.benchmark_datasets.runner import write_json


def audit(result: dict[str, Any]) -> dict[str, Any]:
    """Report measured coverage before drawing conclusions about a provider."""
    entries = {row["dataset_id"]: row for row in result["catalog"]["entries"]}
    groups: dict[tuple[str, str], list[dict[str, Any]]] = defaultdict(list)
    lineage: dict[str, list[str]] = defaultdict(list)
    duplicates: dict[str, list[str]] = defaultdict(list)
    for record in result["records"]:
        entry = entries[record["dataset_id"]]
        groups[(entry["source"], entry["family"])].append(record)
        group = entry["metadata"].get("lineage")
        if group:
            lineage[group].append(record["dataset_id"])
        if record.get("decoded_sha256"):
            duplicates[record["decoded_sha256"]].append(record["dataset_id"])
    summaries = []
    for (source, family), records in sorted(groups.items()):
        counts = Counter(record["status"] for record in records)
        available = Counter()
        methods = Counter()
        query_available = Counter()
        missing = Counter()
        for record in records:
            for key, metric in record.get("metrics", {}).items():
                if metric.get("value") is not None:
                    available[key] += 1
                else:
                    missing[f"{key}:{metric['reason']}"] += 1
                methods[metric["method"]] += 1
            for key, metric in record.get("query_metrics", {}).items():
                if metric.get("value") is not None:
                    query_available[key] += 1
        summaries.append(
            {
                "source": source,
                "family": family,
                "catalog_count": len(records),
                "status_counts": dict(counts),
                "metric_available_counts": dict(available),
                "measurement_methods": dict(methods),
                "metric_unavailable_reasons": dict(missing),
                "query_metric_available_counts": dict(query_available),
            }
        )
    return {
        "families": summaries,
        "discovery_failures": result["catalog"]["discovery_failures"],
        "lineage_groups": {key: ids for key, ids in sorted(lineage.items()) if len(ids) > 1},
        "identical_payload_groups": {
            key: ids for key, ids in sorted(duplicates.items()) if len(ids) > 1
        },
        "claims": {
            "bias_eliminated": False,
            "planner_performance_measured": False,
            "population": result["catalog"]["scope"],
        },
    }


def balanced_cohort(result: dict[str, Any]) -> dict[str, Any]:
    """Suggest equal family/map weights within each compatible semantic class.

    This is an explicit target distribution, not a claim of universal absence of
    bias. Measured and missing strata are both retained. Do not use these weights
    as inference weights for a census of all original queries.
    """
    entries = {row["dataset_id"]: row for row in result["catalog"]["entries"]}
    by_class: dict[str, dict[str, dict[str, list[str]]]] = defaultdict(
        lambda: defaultdict(lambda: defaultdict(list))
    )
    for record in result["records"]:
        entry = entries[record["dataset_id"]]
        semantic_class = record["classification"]["comparison_key"]
        lineage = entry["metadata"].get("lineage", entry["dataset_id"])
        # Mirrors and original/scaled variants share their originating family.
        family = f"{entry['source']}:{entry['family']}"
        if lineage.startswith("movingai:warframe:"):
            family = "movingai:warframe"
        elif lineage.startswith("movingai:baldurs_gate:"):
            family = "movingai:baldurs_gate"
        if entry["metadata"].get("version") == "legacy_2018":
            continue
        by_class[semantic_class][family][lineage].append(record["dataset_id"])
    strata = []
    for semantic_class, families in sorted(by_class.items()):
        for family, maps in sorted(families.items()):
            for lineage, ids in sorted(maps.items()):
                strata.append(
                    {
                        "comparison_key": semantic_class,
                        "family": family,
                        "lineage": lineage,
                        "dataset_ids": sorted(ids),
                        "map_weight": 1 / len(families) / len(maps),
                        "variant_weight": 1 / len(families) / len(maps) / len(ids),
                    }
                )
    return {
        "weighting": "equal_family_then_equal_lineage_within_semantic_class",
        "pooled_across_semantic_classes": False,
        "strata": strata,
        "query_policy": {
            "source_queries": "retain_original_distribution_as_separate_view",
            "balanced_queries": "uniform_hash_within_fixed_displacement_bins",
            "displacement_bins": [0, 0.2, 0.4, 0.6, 0.8, 1.0],
            "reachability": "separate_reachable_unreachable_unknown",
            "heuristic_error": "only_with_independently_validated_cost_and_lower_bound",
            "baseline_expansions": "diagnostic_only_not_primary_selection",
        },
    }


def stratify_queries(
    queries: list[dict[str, Any]], *, per_bin: int = 20, seed: int = 7
) -> dict[str, Any]:
    """Freeze query IDs without conditioning selection on candidate success.

    Input coordinates/displacements must already use a single movement and cost
    profile. Reverse pairs retain direction but remain grouped for inference.
    """
    if per_bin < 1:
        raise ValueError("per_bin must be positive")
    buckets: dict[tuple[str, str, int], list[dict[str, Any]]] = defaultdict(list)
    ids: set[str] = set()
    for query in queries:
        identifier = query["workload_id"]
        distance = query["normalized_displacement"]
        if (
            not isinstance(distance, (float, int))
            or not math.isfinite(distance)
            or not 0 <= distance <= 1
        ):
            raise ValueError("normalized_displacement must be finite and in [0,1]")
        if identifier in ids:
            raise ValueError("Duplicate workload ID")
        ids.add(identifier)
        state = query.get("reference_state", "unknown")
        if state not in {"reachable", "unreachable", "unknown"}:
            raise ValueError("Invalid reference_state")
        buckets[(query["map_id"], state, min(4, int(distance * 5)))].append(query)
    records = []
    selected = []
    for key, rows in sorted(buckets.items()):
        ordered = sorted(
            rows,
            key=lambda row: hashlib.sha256(f"{seed}|{row['workload_id']}".encode()).hexdigest(),
        )
        chosen = [row["workload_id"] for row in ordered[:per_bin]]
        records.append(
            {
                "map_id": key[0],
                "reference_state": key[1],
                "bin": key[2],
                "available": len(rows),
                "selected_ids": chosen,
            }
        )
        selected.extend(chosen)
    return {
        "seed": seed,
        "per_bin": per_bin,
        "selected_ids": selected,
        "strata": records,
        "empty_bins_policy": "never_fill_from_other_bins",
    }


def render_report(result: dict[str, Any], output: Path) -> dict[str, Any]:
    """Write machine-readable audits, cohort weights and a compact human report."""
    summary = audit(result)
    write_json(output / "coverage.json", summary)
    write_json(output / "cohort.json", balanced_cohort(result))
    lines = [
        "# Dataset characterization",
        "",
        "No candidate planner was benchmarked.",
        "",
        "Catalog coverage and metric coverage are different. Missing and sampled values "
        "must not be interpreted as exact population properties.",
        "",
        "| Source | Family | Catalog | Profiled | Not selected | Resource limited | Errors |",
        "|---|---|---:|---:|---:|---:|---:|",
    ]
    for family in summary["families"]:
        counts = family["status_counts"]
        lines.append(
            f"| {family['source']} | {family['family']} | {family['catalog_count']} | "
            f"{counts.get('profiled', 0)} | {counts.get('not_selected', 0)} | "
            f"{counts.get('resource_limited', 0)} | {counts.get('error', 0)} |"
        )
    lines.extend(
        [
            "",
            "## Observed properties",
            "",
            "Values below belong only to the named asset and movement profile.",
            "",
            "| Asset | Metric | Value | Method |",
            "|---|---|---|---|",
        ]
    )
    entries = {row["dataset_id"]: row for row in result["catalog"]["entries"]}
    for record in result["records"]:
        entry = entries[record["dataset_id"]]
        for name in (
            "free_nodes",
            "vertices",
            "arcs_per_vertex",
            "obstacle_fraction",
            "connected_components",
            "strong_components",
            "free_volume_fraction",
        ):
            metric = record.get("metrics", {}).get(name)
            if metric and metric["value"] is not None:
                lines.append(
                    f"| {entry['source']}/{entry['name']} | {name} | "
                    f"{metric['value']} | {metric['method']} |"
                )
    lines.extend(
        [
            "",
            "## Supplied query audit",
            "",
            "Source-recorded costs are unverified. Rows with scaled dimensions are counted separately.",
            "",
            "| Asset | Rows | Duplicate ordered pairs | Scaled rows | Median normalized displacement |",
            "|---|---:|---:|---:|---:|",
        ]
    )
    for record in result["records"]:
        queries = record.get("query_metrics", {})
        if "rows" not in queries:
            continue
        entry = entries[record["dataset_id"]]
        displacement = queries["normalized_displacement"]["value"]["median"]
        lines.append(
            f"| {entry['source']}/{entry['name']} | {queries['rows']['value']} | "
            f"{queries['duplicates']['value']} | {queries.get('scaled_rows', {}).get('value', 0)} | "
            f"{displacement} |"
        )
    lines.extend(["", "## Missing coverage", ""])
    for failure in summary["discovery_failures"]:
        lines.append(f"- Discovery: {failure['collection']}: {failure['error']}")
    for record in result["records"]:
        if record["status"] in {"error", "resource_limited"}:
            entry = entries[record["dataset_id"]]
            lines.append(f"- {entry['source']}/{entry['name']}: {record['error']}")
    lines.extend(
        [
            "",
            "## Interpretation",
            "",
            "- Source family names are provenance labels, not measured topology classes.",
            "- Use family/map-balanced and original-distribution views separately; neither "
            "is unbiased for every application.",
            "- Do not pool discrete graph optima, any-angle path lengths, or robot motion "
            "costs. Preserve movement, robot footprint, metric, objective, and goal policy.",
            "- Keep failed, unknown and resource-limited inputs in coverage denominators. "
            "Never choose workloads from planner success or baseline runtime.",
            "- Cluster uncertainty and tuning splits by map lineage, including mirrors, "
            "scaled versions and shared geographical origins.",
            "- Source-reported costs and supplied paths are not independent optimality proofs.",
            "- Continuous free-volume samples do not prove connectivity or infeasibility.",
            "- See coverage.json for per-metric coverage and cohort.json for explicit weights.",
            "",
        ]
    )
    (output / "report.md").write_text("\n".join(lines))
    return summary
