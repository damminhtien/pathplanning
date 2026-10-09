from __future__ import annotations

from io import StringIO

import numpy as np

from pathplanning.native import NativeGraph
from pathplanning.spaces.grid2d import Grid2DSamplingSpace
from scripts.benchmark_format_cohorts import (
    _barn_planning_circles,
    _coalesce_parallel_arcs,
    _continuous_params,
    _continuous_variants,
    _read_jsonl,
    _run_discrete_case,
    _segment_clear,
    _voxel_scenario_query,
    load_dimacs_csr,
    strict_voxel_csr,
)


def test_barn_cohort_uses_only_model_compatible_planners_and_fine_collision_step():
    planners = _continuous_variants()

    assert "jit_star" not in planners
    assert "hybrid_astar" not in planners
    assert "state_lattice" not in planners
    assert len(planners) == 14
    assert _continuous_params("rrt_star").collision_step == 0.01
    assert _continuous_params("prm_star").collision_step == 0.01
    assert _continuous_params("rrt_star").allow_python_callbacks is False


def test_barn_half_step_inflation_prevents_missed_source_circle_collisions():
    source_circles = [(0.23, 0.0, 0.01)]
    planning_circles, margin = _barn_planning_circles(source_circles, 0.5)
    space = Grid2DSamplingSpace(
        x_range=(-2.0, 2.0),
        y_range=(-2.0, 2.0),
        obs_circle=planning_circles,
        delta=0.0,
        collision_step=0.5,
    )
    first = np.array([[-1.0, 0.0]])
    second = np.array([[1.0, 0.0]])

    assert margin == 0.250000001
    assert not _segment_clear(first, second, source_circles)[0]
    assert not space.is_motion_valid_with_step(first[0], second[0], 0.5)


def test_load_dimacs_csr_preserves_directed_parallel_arcs(tmp_path):
    graph_file = tmp_path / "tiny.gr"
    graph_file.write_text(
        "c tiny directed graph\n"
        "p sp 3 4\n"
        "a 1 2 2\n"
        "a 1 2 5\n"
        "a 2 3 1\n"
        "a 3 1 7\n",
        encoding="ascii",
    )

    offsets, indices, weights = load_dimacs_csr(graph_file)

    assert offsets.tolist() == [0, 2, 3, 4]
    assert indices.tolist() == [1, 1, 2, 0]
    assert weights.tolist() == [2.0, 5.0, 1.0, 7.0]


def test_dimacs_coalescing_keeps_minimum_cost_for_each_directed_pair(tmp_path):
    graph_file = tmp_path / "parallel.gr"
    graph_file.write_text(
        "p sp 3 5\n"
        "a 1 2 5\n"
        "a 1 2 2\n"
        "a 2 3 1\n"
        "a 3 1 7\n"
        "a 2 1 9\n",
        encoding="ascii",
    )

    coalesced = _coalesce_parallel_arcs(*load_dimacs_csr(graph_file))

    assert coalesced[0].tolist() == [0, 1, 3, 4]
    assert coalesced[1].tolist() == [1, 0, 2, 0]
    assert coalesced[2].tolist() == [2.0, 9.0, 1.0, 7.0]


def test_strict_voxel_diagonal_requires_every_proper_side_cell_free():
    blocked = np.zeros((1, 2, 2), dtype=np.bool_)
    offsets, indices, _ = strict_voxel_csr(blocked)
    assert 3 in indices[int(offsets[0]) : int(offsets[1])]

    blocked[0, 1, 0] = True
    offsets, indices, _ = strict_voxel_csr(blocked)
    assert 3 not in indices[int(offsets[0]) : int(offsets[1])]


def test_voxel_query_keeps_source_optimum_and_scenario_line(tmp_path):
    scenario = tmp_path / "tiny.3dscen"
    scenario.write_text(
        "version 1\n"
        "tiny.3dmap\n"
        "0 0 0 1 1 0 1.41421356 1.0\n"
        "0 0 0 1 0 0 1.0 1.0\n",
        encoding="ascii",
    )

    selected = _voxel_scenario_query(
        scenario,
        (2, 2, 1),
        np.zeros((1, 2, 2), dtype=np.bool_),
        seed=7,
    )

    assert selected is not None
    assert selected[0:2] == (0, 3)
    assert selected[3] == {
        "scenario_line": 3,
        "source_scenario_optimum": 1.41421356,
        "source_scenario_difficulty": 1.0,
        "movement_semantics": "strict_26_neighbor_euclidean_no_corner_cutting",
    }


def test_jsonl_resume_discards_only_a_truncated_final_record(tmp_path):
    output = tmp_path / "runs.jsonl"
    output.write_bytes(b'{"complete":true}\n{"truncated":')

    assert _read_jsonl(output) == [{"complete": True}]
    assert output.read_bytes() == b'{"complete":true}\n'


def test_format_runner_executes_and_resumes_all_generic_discrete_variants():
    offsets = np.array([0, 2, 3, 4], dtype=np.uint64)
    indices = np.array([1, 2, 2, 0], dtype=np.uint64)
    weights = np.array([1.0, 3.0, 1.0, 1.0], dtype=np.float64)
    graph = NativeGraph.from_csr(offsets, indices, weights)
    dataset = {
        "dataset_id": "tiny-dataset",
        "source": "test",
        "family": "directed_weighted_graph",
        "name": "tiny.gr",
    }
    source_assets = [{"sha256": "0" * 64}]
    completed: set[tuple[str, str]] = set()
    stream = StringIO()
    try:
        first = _run_discrete_case(
            dataset=dataset,
            cohort="tiny_directed_weighted",
            query_id="tiny-query",
            graph=graph,
            indptr=offsets,
            indices=indices,
            weights=weights,
            start=0,
            goal=2,
            source_assets=source_assets,
            seed=7,
            completed=completed,
            output_stream=stream,
        )
        second = _run_discrete_case(
            dataset=dataset,
            cohort="tiny_directed_weighted",
            query_id="tiny-query",
            graph=graph,
            indptr=offsets,
            indices=indices,
            weights=weights,
            start=0,
            goal=2,
            source_assets=source_assets,
            seed=7,
            completed=completed,
            output_stream=stream,
        )
    finally:
        graph.close()

    assert len(first) == 13
    assert second == []
    assert all(row["outcome"]["status"] != "error" for row in first)
    assert len(stream.getvalue().splitlines()) == 13
