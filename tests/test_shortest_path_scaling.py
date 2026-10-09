from __future__ import annotations

import numpy as np
import pytest

from scripts.shortest_path_benchmark.scaling import (
    ablation_factor,
    connectivity_summary,
    fit_loglog_slope,
    heuristic_variants,
    sample_queries,
)


def test_connectivity_and_sampled_queries_are_reproducible() -> None:
    blocked = np.zeros((32, 32), dtype=np.bool_)
    blocked[:, 16] = True
    components, largest = connectivity_summary(blocked)
    assert (components, largest) == (2, 512)
    first = sample_queries(blocked, seed=7, per_bin=1)
    second = sample_queries(blocked, seed=7, per_bin=1)
    assert first == second
    assert {query.displacement_bin for query in first} == set(range(5))


def test_consistent_heuristic_ablations_and_slope_ci() -> None:
    variants = heuristic_variants()
    assert [variant["alpha"] for variant in variants] == [0, 0.25, 0.5, 0.75, 1]
    estimate = fit_loglog_slope([(10, 100), (20, 400), (40, 1600)], draws=100, seed=7)
    assert estimate["slope"] == pytest.approx(2.0)
    assert (
        estimate["ci95"]
        == fit_loglog_slope([(10, 100), (20, 400), (40, 1600)], draws=100, seed=7)["ci95"]
    )
    assert estimate["interpretation"] == "empirical_trend_only"
    assert (
        ablation_factor({"h": 1, "weight": 1}, {"h": 0.5, "weight": 1}, factor="h")["factor"] == "h"
    )
    with pytest.raises(ValueError, match="only h"):
        ablation_factor({"h": 1, "weight": 1}, {"h": 0.5, "weight": 2}, factor="h")
