"""Coverage-aware analysis for single-agent shortest-path campaigns.

All comparisons use measured observations from the same pass and protocol.
Latency is summarized within each query before queries are compared.  The
reference result defines eligibility independently of candidate success.
"""

from __future__ import annotations

from collections import defaultdict
import hashlib
import html
import json
import math
import os
from pathlib import Path
import random
import statistics
import tempfile
from typing import Any, Iterable, Mapping

ANALYSIS_VERSION = "shortest_path_analysis_v3"
_COMPLETE_STATUSES = frozenset({"completed", "ok", "success"})
_MEASURED_PHASES = frozenset({"measured", "measure"})
_PROTOCOL_FIELDS = ("scope", "graph_state", "protocol_hash", "budget")


def _finite_number(value: Any) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    number = float(value)
    return number if math.isfinite(number) else None


def _numeric_metric(value: Any) -> int | float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def _field(record: Mapping[str, Any], name: str) -> Any:
    value = record.get(name)
    if value is not None:
        return value
    provenance = record.get("provenance")
    return provenance.get(name) if isinstance(provenance, dict) else None


def _protocol(record: Mapping[str, Any]) -> tuple[str, ...]:
    values = [str(record.get("pass", "unknown"))]
    values.extend(str(_field(record, key) or "") for key in _PROTOCOL_FIELDS)
    input_data = record.get("input") or {}
    values.append(str(input_data.get("movement_profile") or input_data.get("profile") or ""))
    return tuple(values)


def _percentile(values: Iterable[int | float], percent: float) -> int | float | None:
    ordered = sorted(values)
    if not ordered:
        return None
    position = (len(ordered) - 1) * percent / 100
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)


def _stats(values: Iterable[int | float]) -> dict[str, float | int | None]:
    clean = [value for value in values if _numeric_metric(value) is not None]
    return {
        "count": len(clean),
        "median": statistics.median(clean) if clean else None,
        "p95": _percentile(clean, 95),
        "min": min(clean) if clean else None,
        "max": max(clean) if clean else None,
        "iqr": (
            _percentile(clean, 75) - _percentile(clean, 25) if clean else None  # type: ignore[operator]
        ),
    }


def _read_jsonl(path: Path) -> list[dict[str, Any]]:
    if not path.exists():
        return []
    rows: list[dict[str, Any]] = []
    with path.open(encoding="utf-8") as source:
        for line_number, line in enumerate(source, 1):
            if not line.strip():
                continue
            try:
                row = json.loads(line)
            except json.JSONDecodeError as exc:
                raise ValueError(f"Invalid JSONL at {path}:{line_number}") from exc
            if not isinstance(row, dict):
                raise ValueError(f"Expected object at {path}:{line_number}")
            rows.append(row)
    return rows


def _reference_state(row: Mapping[str, Any]) -> tuple[str, float | None]:
    input_data = row.get("input") or {}
    outcome = row.get("outcome") or {}
    for data in (input_data, outcome, row):
        for key in ("reference_cost", "oracle_cost"):
            cost = _finite_number(data.get(key))
            if cost is not None:
                if cost < 0:
                    raise ValueError("Reference cost cannot be negative")
                return "solvable", cost
    for data in (input_data, outcome, row):
        if data.get("reference_reachable") is False or data.get("oracle_status") in {
            "unreachable",
            "no_path",
        }:
            return "unreachable", None
    if outcome.get("execution_status") == "proved_unreachable":
        return "unreachable", None
    return "unknown", None


def _collect_reference(
    manifest: Mapping[str, Any], oracle: list[dict[str, Any]], runs: list[dict[str, Any]]
) -> dict[str, dict[str, Any]]:
    reference: dict[str, dict[str, Any]] = {}
    candidates: list[Mapping[str, Any]] = []
    for key in ("workloads", "queries", "cases"):
        rows = manifest.get(key, [])
        if isinstance(rows, list):
            candidates.extend(row for row in rows if isinstance(row, dict))
    candidates.extend(oracle)
    candidates.extend(runs)
    for row in candidates:
        workload_id = row.get("workload_id")
        if not isinstance(workload_id, str) or not workload_id:
            continue
        input_data = row.get("input") or row
        state, cost = _reference_state(row)
        current = reference.setdefault(
            workload_id,
            {
                "state": "unknown",
                "cost": None,
                "family": input_data.get("family") or "unknown",
                "map": input_data.get("map_sha256") or input_data.get("map_path") or workload_id,
            },
        )
        if current["family"] == "unknown" and input_data.get("family"):
            current["family"] = input_data["family"]
        if current["map"] == workload_id:
            current["map"] = (
                input_data.get("map_sha256") or input_data.get("map_path") or workload_id
            )
        if state == "unknown":
            continue
        if current["state"] != "unknown" and current["state"] != state:
            raise ValueError(f"Conflicting reference reachability: {workload_id}")
        old_cost = current["cost"]
        if (
            cost is not None
            and old_cost is not None
            and not math.isclose(old_cost, cost, rel_tol=1e-10, abs_tol=1e-8)
        ):
            raise ValueError(f"Conflicting reference cost: {workload_id}")
        current["state"] = state
        current["cost"] = cost
    return reference


def _eligible_ids(
    manifest: Mapping[str, Any], pass_name: str, observed: set[str], reference: Mapping[str, Any]
) -> tuple[set[str], str]:
    cohorts = manifest.get("cohorts") or {}
    pass_cohort = cohorts.get(pass_name, {}) if isinstance(cohorts, dict) else {}
    if isinstance(pass_cohort, dict) and isinstance(pass_cohort.get("workload_ids"), list):
        return set(pass_cohort["workload_ids"]), "manifest_cohort"
    pass_key = f"{pass_name}_workload_ids"
    if isinstance(manifest.get(pass_key), list):
        return set(manifest[pass_key]), "manifest_cohort"
    workload_rows = manifest.get("workloads") or manifest.get("queries") or manifest.get("cases")
    if pass_name == "work" and isinstance(workload_rows, list):
        return set(reference), "manifest_workloads"
    return observed, "observed_union"


def _valid_solved(row: Mapping[str, Any]) -> bool:
    outcome = row.get("outcome") or {}
    execution = outcome.get("execution_status")
    return (
        row.get("status") in _COMPLETE_STATUSES
        and (execution is None or execution in _COMPLETE_STATUSES
             or execution in {"valid_optimal", "valid_suboptimal"})
        and outcome.get("path_present") is True
        and outcome.get("path_valid") is True
        and _finite_number(outcome.get("path_cost")) is not None
    )


def _decision_correct(row: Mapping[str, Any]) -> bool:
    outcome = row.get("outcome") or {}
    return (
        row.get("status") in _COMPLETE_STATUSES
        and outcome.get("path_present") is False
        and (outcome.get("execution_status") == "proved_unreachable"
             or outcome.get("planner_stop_reason") in {"no_progress", "unreachable", "no_path"})
    )


def _signature(row: Mapping[str, Any]) -> tuple[Any, ...]:
    outcome = row.get("outcome") or {}
    return (
        row.get("status"),
        outcome.get("execution_status"),
        outcome.get("planner_stop_reason"),
        outcome.get("path_present"),
        outcome.get("path_valid"),
        outcome.get("path_cost"),
        outcome.get("path_hash"),
        outcome.get("iters"),
        outcome.get("nodes"),
    )


def _query_observation(rows: list[dict[str, Any]], pass_name: str) -> dict[str, Any]:
    consistent = len({_signature(row) for row in rows}) <= 1
    first = rows[0]
    valid = consistent and _valid_solved(first)
    timings: list[float] = []
    if pass_name == "latency" and valid:
        for row in rows:
            timing = row.get("timing") or {}
            field = "native_call_s" if row.get("scope") == "prepared_kernel" else "api_total_s"
            elapsed = _finite_number(timing.get(field))
            if elapsed is not None and elapsed > 0:
                timings.append(elapsed)
    return {
        "consistent": consistent,
        "valid_solved": valid,
        "row": first,
        "time_s": statistics.median(timings) if len(timings) == len(rows) and timings else None,
        "within_query_iqr_s": (
            _percentile(timings, 75) - _percentile(timings, 25)  # type: ignore[operator]
            if len(timings) == len(rows) and timings
            else None
        ),
        "attempt_count": len(rows),
        "timeout_count": sum(row.get("status") == "timeout" for row in rows),
        "error_count": sum(
            row.get("status") not in _COMPLETE_STATUSES | {"timeout"} for row in rows
        ),
    }


def clustered_bootstrap_speedup(
    pairs: list[tuple[str, float]], *, draws: int = 2000, seed: int = 7
) -> dict[str, Any]:
    """Resample maps, then paired queries within each sampled map."""
    if draws < 1:
        raise ValueError("draws must be positive")
    clusters: dict[str, list[float]] = defaultdict(list)
    for map_id, speedup in pairs:
        if not math.isfinite(speedup) or speedup <= 0:
            raise ValueError("Speedups must be finite and positive")
        clusters[map_id].append(speedup)
    if not clusters:
        return {
            "draws": draws,
            "seed": seed,
            "map_count": 0,
            "median_ci95": None,
            "geomean_ci95": None,
            "few_maps": True,
        }
    rng = random.Random(seed)
    map_ids = sorted(clusters)
    median_draws: list[float] = []
    geomean_draws: list[float] = []
    for _ in range(draws):
        sampled: list[float] = []
        for _ in map_ids:
            selected = clusters[rng.choice(map_ids)]
            sampled.extend(rng.choice(selected) for _ in selected)
        median_draws.append(statistics.median(sampled))
        geomean_draws.append(math.exp(statistics.mean(math.log(value) for value in sampled)))
    return {
        "draws": draws,
        "seed": seed,
        "map_count": len(map_ids),
        "median_ci95": [_percentile(median_draws, 2.5), _percentile(median_draws, 97.5)],
        "geomean_ci95": [_percentile(geomean_draws, 2.5), _percentile(geomean_draws, 97.5)],
        "few_maps": len(map_ids) < 3,
    }


def preprocessing_break_even(
    prep_a_s: float, query_a_s: float, prep_b_s: float, query_b_s: float
) -> dict[str, Any]:
    """Classify when P_B + Q*q_B is no greater than P_A + Q*q_A, Q >= 0."""
    values = (prep_a_s, query_a_s, prep_b_s, query_b_s)
    if any(not math.isfinite(value) or value < 0 for value in values):
        raise ValueError("Preparation and query costs must be finite and nonnegative")
    prep_delta = prep_b_s - prep_a_s
    query_saving = query_a_s - query_b_s
    if prep_delta == 0 and query_saving == 0:
        classification, queries = "equal_at_all_query_counts", 0
    elif prep_delta <= 0 and query_saving >= 0:
        classification, queries = "b_dominates", 0
    elif prep_delta > 0 and query_saving <= 0:
        classification, queries = "b_never_breaks_even", None
    elif prep_delta > 0:
        classification, queries = (
            "b_breaks_even_after_queries",
            math.ceil(prep_delta / query_saving),
        )
    elif prep_delta == 0:
        classification, queries = "b_only_equal_at_zero", 0
    else:
        classification, queries = "b_wins_below_threshold", None
    return {
        "classification": classification,
        "first_query_count_b_wins": queries,
        "prep_delta_s": prep_delta,
        "per_query_saving_s": query_saving,
        "crossing_queries": abs(prep_delta / query_saving) if query_saving != 0 else None,
    }


def _metric_stats(observations: Mapping[str, dict[str, Any]], namespace: str) -> dict[str, Any]:
    values: dict[str, list[int | float]] = defaultdict(list)
    for observation in observations.values():
        if not observation["consistent"] or observation["row"].get("status") not in _COMPLETE_STATUSES:
            continue
        row = observation["row"]
        for name, value in (row.get(namespace) or {}).items():
            number = _numeric_metric(value)
            if number is not None:
                values[name].append(number)
    return {name: _stats(samples) for name, samples in sorted(values.items())}


def _paired_metric_ratios(
    baseline: Mapping[str, dict[str, Any]], candidate: Mapping[str, dict[str, Any]], namespace: str
) -> dict[str, Any]:
    values: dict[str, list[float]] = defaultdict(list)
    for workload_id in baseline.keys() & candidate.keys():
        left, right = baseline[workload_id], candidate[workload_id]
        if not left["valid_solved"] or not right["valid_solved"]:
            continue
        base = left["row"].get(namespace) or {}
        cand = right["row"].get(namespace) or {}
        for name in base.keys() & cand.keys():
            base_value = _finite_number(base[name])
            cand_value = _finite_number(cand[name])
            if base_value is not None and cand_value is not None and cand_value > 0:
                values[name].append(base_value / cand_value)
    return {name: _stats(samples) for name, samples in sorted(values.items())}


def _quality(
    observations: Mapping[str, dict[str, Any]], reference: Mapping[str, dict[str, Any]]
) -> dict[str, Any]:
    ratios: list[float] = []
    reference_sum = 0.0
    candidate_sum = 0.0
    zero_optimum_count = 0
    optimal_count = 0
    for workload_id, observation in observations.items():
        state = reference.get(workload_id, {})
        if state.get("state") != "solvable" or not observation["valid_solved"]:
            continue
        optimum = state["cost"]
        cost = _finite_number((observation["row"].get("outcome") or {}).get("path_cost"))
        if cost is None:
            continue
        if math.isclose(cost, optimum, rel_tol=1e-10, abs_tol=1e-8):
            optimal_count += 1
        candidate_sum += cost
        reference_sum += optimum
        if optimum == 0:
            zero_optimum_count += 1
        else:
            ratios.append(cost / optimum)
    return {
        "solved_with_known_optimum_count": len(ratios) + zero_optimum_count,
        "optimal_count": optimal_count,
        "zero_optimum_count": zero_optimum_count,
        "mean_cost_ratio": statistics.mean(ratios) if ratios else None,
        "mean_cost_ratio_denominator": len(ratios),
        "sum_cost_ratio": candidate_sum / reference_sum if reference_sum > 0 else None,
        "sum_cost_ratio_reference_total": reference_sum,
        "sum_cost_ratio_candidate_total": candidate_sum,
    }


def _group_analysis(
    rows: list[dict[str, Any]],
    manifest: Mapping[str, Any],
    reference: dict[str, dict[str, Any]],
    *,
    baseline: str,
    bootstrap_draws: int,
    bootstrap_seed: int,
) -> dict[str, Any]:
    pass_name = str(rows[0].get("pass", "unknown"))
    protocol = _protocol(rows[0])
    observed = {row["workload_id"] for row in rows}
    eligible, denominator_source = _eligible_ids(manifest, pass_name, observed, reference)
    solvable = {key for key in eligible if reference.get(key, {}).get("state") == "solvable"}
    unreachable = {key for key in eligible if reference.get(key, {}).get("state") == "unreachable"}
    grouped: dict[str, dict[str, list[dict[str, Any]]]] = defaultdict(lambda: defaultdict(list))
    for row in rows:
        variant_name = row.get("variant_name")
        variant_id = row["variant_id"]
        if not isinstance(variant_name, str):
            variant_name = next(
                (
                    item.get("variant_name", item.get("id"))
                    for item in manifest.get("variants", [])
                    if isinstance(item, dict) and item.get("variant_id") == variant_id
                ),
                variant_id,
            )
        grouped[variant_name][row["workload_id"]].append(row)
    for variant in manifest.get("variants", []):
        variant_name = (
            variant.get("variant_name", variant.get("id"))
            if isinstance(variant, dict)
            else variant
        )
        if isinstance(variant_name, str):
            grouped[variant_name]
    observations = {
        variant: {key: _query_observation(group, pass_name) for key, group in cases.items()}
        for variant, cases in grouped.items()
    }
    variants: dict[str, Any] = {}
    for variant, cases in sorted(observations.items()):
        valid = {key for key in solvable if key in cases and cases[key]["valid_solved"]}
        correct_unreachable = {
            key
            for key in unreachable
            if key in cases and cases[key]["consistent"] and _decision_correct(cases[key]["row"])
        }
        times = [cases[key]["time_s"] for key in valid if cases[key]["time_s"] is not None]
        iqr = [
            cases[key]["within_query_iqr_s"]
            for key in valid
            if cases[key]["within_query_iqr_s"] is not None
        ]
        by_map: dict[str, list[float]] = defaultdict(list)
        by_family: dict[str, list[float]] = defaultdict(list)
        for key in valid:
            if cases[key]["time_s"] is not None:
                by_map[str(reference[key]["map"])].append(cases[key]["time_s"])
                by_family[str(reference[key]["family"])].append(cases[key]["time_s"])
        variants[variant] = {
            "coverage": {
                "valid_solved_count": len(valid),
                "oracle_solvable_count": len(solvable),
                "rate": len(valid) / len(solvable) if solvable else None,
                "denominator_source": denominator_source,
                "unreachable_correct_count": len(correct_unreachable),
                "oracle_unreachable_count": len(unreachable),
                "unreachable_decision_accuracy": (
                    len(correct_unreachable) / len(unreachable) if unreachable else None
                ),
                "unknown_reference_count": len(eligible - solvable - unreachable),
            },
            "observations": {
                "unique_queries": len(cases),
                "measured_invocations": sum(item["attempt_count"] for item in cases.values()),
                "timeouts": sum(item["timeout_count"] for item in cases.values()),
                "errors": sum(item["error_count"] for item in cases.values()),
                "inconsistent_queries": sorted(
                    key for key, item in cases.items() if not item["consistent"]
                ),
                "invalid_path_queries": sorted(
                    key
                    for key, item in cases.items()
                    if (item["row"].get("outcome") or {}).get("path_valid") is False
                ),
            },
            "timing": {
                "query_medians_s": _stats(times),
                "median_within_query_iqr_s": statistics.median(iqr) if iqr else None,
                "map_macro_median_s": statistics.mean(statistics.median(x) for x in by_map.values())
                if by_map
                else None,
                "map_macro_count": len(by_map),
                "family_macro_median_s": statistics.mean(
                    statistics.median(x) for x in by_family.values()
                )
                if by_family
                else None,
                "family_macro_count": len(by_family),
            },
            "quality": _quality(cases, reference),
            "work": _metric_stats(cases, "work") if pass_name == "work" else {},
            # The work pass owns allocator instrumentation, while the memory
            # pass owns fresh-worker RSS. Keep both under their original pass
            # so the two measurement boundaries remain visible.
            "memory": _metric_stats(cases, "memory") if pass_name in {"work", "memory"} else {},
        }
    for variant, cases in observations.items():
        if variant == baseline or baseline not in observations:
            continue
        base_cases = observations[baseline]
        paired: list[tuple[str, float]] = []
        common_valid = 0
        for key in sorted(base_cases.keys() & cases.keys() & solvable):
            left, right = base_cases[key], cases[key]
            if not left["valid_solved"] or not right["valid_solved"]:
                continue
            common_valid += 1
            if left["time_s"] is None or right["time_s"] is None:
                continue
            paired.append((str(reference[key]["map"]), left["time_s"] / right["time_s"]))
        ratios = [value for _, value in paired]
        variants[variant]["paired_vs_baseline"] = {
            "baseline": baseline,
            "common_valid_solved_count": common_valid,
            "timed_pair_count": len(paired),
            "median_speedup": statistics.median(ratios) if ratios else None,
            "geometric_mean_speedup": (
                math.exp(statistics.mean(math.log(value) for value in ratios)) if ratios else None
            ),
            "bootstrap": clustered_bootstrap_speedup(
                paired, draws=bootstrap_draws, seed=bootstrap_seed
            ),
            "work_baseline_over_candidate": (
                _paired_metric_ratios(base_cases, cases, "work") if pass_name == "work" else {}
            ),
            "memory_baseline_over_candidate": (
                _paired_metric_ratios(base_cases, cases, "memory")
                if pass_name in {"work", "memory"}
                else {}
            ),
        }
    return {
        "pass": pass_name,
        "protocol": dict(zip(("pass", *_PROTOCOL_FIELDS, "movement_profile"), protocol)),
        "eligibility": {
            "source": denominator_source,
            "workload_count": len(eligible),
            "oracle_solvable_count": len(solvable),
            "oracle_unreachable_count": len(unreachable),
        },
        "variants": variants,
    }


def summarize_campaign(
    runs: list[dict[str, Any]],
    *,
    manifest: Mapping[str, Any] | None = None,
    oracle: list[dict[str, Any]] | None = None,
    baseline: str = "dijkstra",
    bootstrap_draws: int = 2000,
    bootstrap_seed: int = 7,
) -> dict[str, Any]:
    """Summarize raw JSONL observations without dropping failed invocations."""
    if bootstrap_draws < 1:
        raise ValueError("bootstrap_draws must be positive")
    manifest = manifest or {}
    reference = _collect_reference(manifest, oracle or [], runs)
    groups: dict[tuple[str, ...], list[dict[str, Any]]] = defaultdict(list)
    for row in runs:
        if row.get("phase") not in _MEASURED_PHASES:
            continue
        if not isinstance(row.get("workload_id"), str) or not isinstance(
            row.get("variant_id"), str
        ):
            raise ValueError("Measured runs require workload_id and variant_id")
        groups[_protocol(row)].append(row)
    cohorts = [
        _group_analysis(
            rows,
            manifest,
            reference,
            baseline=baseline,
            bootstrap_draws=bootstrap_draws,
            bootstrap_seed=bootstrap_seed,
        )
        for _, rows in sorted(groups.items())
    ]
    return {
        "analysis_version": ANALYSIS_VERSION,
        "baseline": baseline,
        "raw_run_count": len(runs),
        "measured_run_count": sum(map(len, groups.values())),
        "reference_workload_count": len(reference),
        "cohorts": cohorts,
    }


def _atomic_text(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary_name: str | None = None
    try:
        with tempfile.NamedTemporaryFile(
            "w",
            encoding="utf-8",
            dir=path.parent,
            prefix=f".{path.name}.",
            suffix=".tmp",
            delete=False,
        ) as output:
            temporary_name = output.name
            output.write(content)
            output.flush()
            os.fsync(output.fileno())
        os.replace(temporary_name, path)
    except BaseException:
        if temporary_name:
            Path(temporary_name).unlink(missing_ok=True)
        raise


def _number(value: float | None, *, digits: int = 4) -> str:
    return "—" if value is None else f"{value:.{digits}g}"


def _metric_percentiles(
    variant: Mapping[str, Any], namespace: str, metric: str, *, scale: float = 1.0
) -> str:
    stats = (variant.get(namespace) or {}).get(metric)
    if not isinstance(stats, dict):
        return "—"
    median = _finite_number(stats.get("median"))
    p95 = _finite_number(stats.get("p95"))
    if median is None or p95 is None:
        return "—"
    return f"{_number(median / scale)} / {_number(p95 / scale)}"


def _plot_svg(cohort: Mapping[str, Any]) -> str | None:
    variants = cohort["variants"]
    entries = [
        (
            name,
            item["timing"]["query_medians_s"]["median"],
            item["timing"]["query_medians_s"]["count"],
        )
        for name, item in variants.items()
    ]
    entries = [(name, value, count) for name, value, count in entries if value is not None]
    if not entries:
        return None
    width = 800
    height = 90 + 44 * len(entries)
    maximum = max(value for _, value, _ in entries)
    lines = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" '
        f'viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="white"/>',
        '<text x="20" y="30" font-family="sans-serif" font-size="16">'
        "Median per-query time (seconds; valid solved queries)</text>",
    ]
    for index, (name, value, count) in enumerate(entries):
        y = 59 + index * 44
        bar_width = 440 * value / maximum if maximum else 0
        lines.append(
            f'<text x="20" y="{y + 15}" font-family="sans-serif" font-size="13">'
            f"{html.escape(name)}</text>"
        )
        lines.append(f'<rect x="200" y="{y}" width="{bar_width:.1f}" height="20" fill="#286eab"/>')
        lines.append(
            f'<text x="{215 + bar_width:.1f}" y="{y + 15}" '
            f'font-family="sans-serif" font-size="12">{value:.4g} s (n={count})</text>'
        )
    lines.append("</svg>")
    return "\n".join(lines) + "\n"


def _report_markdown(summary: Mapping[str, Any], plots: Mapping[int, str]) -> str:
    lines = [
        "# Single-agent shortest-path evaluation",
        "",
        f"Analysis version: `{summary['analysis_version']}`. Raw observations: "
        f"{summary['raw_run_count']}; measured observations: {summary['measured_run_count']}.",
        "",
        "Coverage uses unique oracle-solvable queries. Latency uses the median of measured release "
        "repeats per valid solved query; work counters and memory measurements come from separate "
        "passes. Timeout and error observations remain in counts.",
        "",
    ]
    for index, cohort in enumerate(summary["cohorts"]):
        protocol = cohort["protocol"]
        pass_name = protocol["pass"]
        lines.extend(
            [
                f"## {pass_name}: {protocol['scope']} / {protocol['graph_state']}",
                "",
                f"Eligibility: {cohort['eligibility']['source']} "
                f"({cohort['eligibility']['oracle_solvable_count']} solvable, "
                f"{cohort['eligibility']['oracle_unreachable_count']} unreachable).",
                "",
            ]
        )
        if pass_name == "latency":
            lines.extend(
                [
                    "| Variant | Valid / solvable | Median s | P95 s | Speedup vs baseline | "
                    "Mean cost ratio | Timeouts | Inconsistent |",
                    "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
                ]
            )
            for name, variant in cohort["variants"].items():
                coverage = variant["coverage"]
                timing = variant["timing"]["query_medians_s"]
                paired = variant.get("paired_vs_baseline", {})
                lines.append(
                    f"| {name} | {coverage['valid_solved_count']} / "
                    f"{coverage['oracle_solvable_count']} | {_number(timing['median'])} | "
                    f"{_number(timing['p95'])} | {_number(paired.get('median_speedup'))} | "
                    f"{_number(variant['quality']['mean_cost_ratio'])} | "
                    f"{variant['observations']['timeouts']} | "
                    f"{len(variant['observations']['inconsistent_queries'])} |"
                )
        else:
            lines.extend(
                [
                    "| Variant | Valid / solvable | Optimal / known optimum | Mean cost ratio | "
                    "Timeouts | Errors | Inconsistent |",
                    "| --- | ---: | ---: | ---: | ---: | ---: | ---: |",
                ]
            )
            for name, variant in cohort["variants"].items():
                coverage = variant["coverage"]
                quality = variant["quality"]
                observations = variant["observations"]
                lines.append(
                    f"| {name} | {coverage['valid_solved_count']} / "
                    f"{coverage['oracle_solvable_count']} | {quality['optimal_count']} / "
                    f"{quality['solved_with_known_optimum_count']} | "
                    f"{_number(quality['mean_cost_ratio'])} | {observations['timeouts']} | "
                    f"{observations['errors']} | "
                    f"{len(observations['inconsistent_queries'])} |"
                )
            if pass_name == "work":
                work_metrics = [
                    ("expanded", "Expanded"),
                    ("edges_examined", "Edges examined"),
                    ("relaxation_attempts", "Relax attempts"),
                    ("relaxation_successes_first", "First discoveries"),
                    ("relaxation_successes_improved", "Improved discoveries"),
                    ("frontier_pushes", "OPEN pushes"),
                    ("frontier_pops", "OPEN pops"),
                    ("stale_pops", "Stale pops"),
                    ("frontier_peak_entries", "Peak OPEN entries"),
                    ("heuristic_lookups", "Heuristic lookups"),
                    ("heuristic_computations", "Heuristic computations"),
                ]
                lines.extend(
                    [
                        "",
                        "### Search work per query (median / P95)",
                        "",
                        "| Variant | " + " | ".join(label for _, label in work_metrics) + " |",
                        "| --- | " + " | ".join("---:" for _ in work_metrics) + " |",
                    ]
                )
                for name, variant in cohort["variants"].items():
                    values = [
                        _metric_percentiles(variant, "work", key)
                        for key, _ in work_metrics
                    ]
                    lines.append("| " + name + " | " + " | ".join(values) + " |")
                allocation_metrics = [
                    ("query_workspace_peak_bytes", "Query workspace MiB"),
                    ("state_capacity_bytes_peak", "Search state MiB"),
                    ("frontier_capacity_bytes_peak", "Frontier MiB"),
                    ("path_workspace_bytes_peak", "Path workspace MiB"),
                    ("result_path_bytes", "Result path MiB"),
                ]
                lines.extend(
                    [
                        "",
                        "### Instrumented query memory (median / P95 MiB)",
                        "",
                        "| Variant | " + " | ".join(label for _, label in allocation_metrics) + " |",
                        "| --- | " + " | ".join("---:" for _ in allocation_metrics) + " |",
                    ]
                )
                for name, variant in cohort["variants"].items():
                    values = [
                        _metric_percentiles(variant, "memory", key, scale=1024 * 1024)
                        for key, _ in allocation_metrics
                    ]
                    lines.append("| " + name + " | " + " | ".join(values) + " |")
            elif pass_name == "memory":
                memory_metrics = [
                    ("process_peak_rss_bytes", "Process RSS MiB"),
                    ("input_occupancy_bytes", "Input occupancy MiB"),
                    ("base_csr_capacity_bytes_after", "Base CSR MiB"),
                    ("reverse_csr_capacity_bytes_after", "Reverse CSR MiB"),
                    ("worker_retained_graph_bytes", "Retained graph MiB"),
                ]
                lines.extend(
                    [
                        "",
                        "### Fresh-worker memory (median / P95 MiB)",
                        "",
                        "| Variant | " + " | ".join(label for _, label in memory_metrics) + " |",
                        "| --- | " + " | ".join("---:" for _ in memory_metrics) + " |",
                    ]
                )
                for name, variant in cohort["variants"].items():
                    values = [
                        _metric_percentiles(variant, "memory", key, scale=1024 * 1024)
                        for key, _ in memory_metrics
                    ]
                    lines.append("| " + name + " | " + " | ".join(values) + " |")
        lines.append("")
        if index in plots:
            lines.extend([f"![Median time by variant]({plots[index]})", ""])
        if pass_name == "latency":
            lines.append(
                "Speedup intervals resample maps and then paired queries within maps; "
                "intervals from fewer than three maps are flagged in summary.json. "
                "Latency uses the release library without metrics instrumentation."
            )
        elif pass_name == "work":
            lines.append(
                "Work counters and tracked query allocations come from the separate metrics build; "
                "full counters are in [summary.json](summary.json). The metrics call is checked "
                "against the production result, and does not contribute to latency timings."
            )
        elif pass_name == "memory":
            lines.append(
                "RSS is the process lifetime high-water from a fresh worker and includes Python, "
                "NumPy, map loading, and graph creation; it is not query-only memory. Input and "
                "retained graph storage are reported separately. Query state/frontier allocation "
                "comes from the work pass. Full metrics are in [summary.json](summary.json)."
            )
        lines.append("")
    return "\n".join(lines)


def analyze_campaign(
    campaign_dir: str | Path,
    *,
    baseline: str = "dijkstra",
    bootstrap_draws: int = 2000,
    bootstrap_seed: int = 7,
) -> dict[str, Any]:
    """Read campaign JSONL and write summary.json, report.md, and simple SVG plots."""
    directory = Path(campaign_dir)
    manifest_path = directory / "manifest.json"
    manifest = (
        json.loads(manifest_path.read_text(encoding="utf-8")) if manifest_path.exists() else {}
    )
    runs = _read_jsonl(directory / "runs.jsonl")
    oracle = _read_jsonl(directory / "oracle.jsonl")
    summary = summarize_campaign(
        runs,
        manifest=manifest,
        oracle=oracle,
        baseline=baseline,
        bootstrap_draws=bootstrap_draws,
        bootstrap_seed=bootstrap_seed,
    )
    plots: dict[int, str] = {}
    for index, cohort in enumerate(summary["cohorts"]):
        image = _plot_svg(cohort)
        if image is None:
            continue
        digest = hashlib.sha256(
            json.dumps(cohort["protocol"], sort_keys=True).encode()
        ).hexdigest()[:10]
        relative = f"plots/latency_{digest}.svg"
        _atomic_text(directory / relative, image)
        plots[index] = relative
    _atomic_text(
        directory / "summary.json",
        json.dumps(summary, indent=2, sort_keys=True, allow_nan=False) + "\n",
    )
    _atomic_text(directory / "report.md", _report_markdown(summary, plots))
    return summary


__all__ = [
    "analyze_campaign",
    "clustered_bootstrap_speedup",
    "preprocessing_break_even",
    "summarize_campaign",
]
