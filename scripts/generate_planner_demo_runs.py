#!/usr/bin/env python3
"""Generate replayable, package-backed runs used by the learning site."""

from __future__ import annotations

import argparse
from collections.abc import Sequence
import hashlib
from importlib.metadata import version
import json
import math
from pathlib import Path
import sys
from typing import Any

import numpy as np

from pathplanning.api import plan_continuous, plan_discrete
from pathplanning.core.contracts import ContinuousProblem, DiscreteProblem, GoalState
from pathplanning.core.params import RrtParams
from pathplanning.core.trace import PlannerTrace, TraceOptions
from pathplanning.planners.search.dstar_lite import DStarLitePlanner
from pathplanning.spaces.grid2d import Grid2DSamplingSpace, Grid2DSearchSpace
from scripts.shortest_path_benchmark.workloads import load_map

ROOT = Path(__file__).resolve().parents[1]
INPUTS = ROOT / "website" / "data" / "planner-demo-inputs"
DEFAULT_OUTPUT = ROOT / "website" / "data" / "planner-demo-runs.json"
TRACE_LIMIT_BYTES = 64 * 1024 * 1024

# Values are part of the stable native trace ABI in pathplanning/native/trace_engine.h.
TRACE_KIND = {
    1: "discover",
    2: "expand",
    3: "parent",
    4: "sample",
    5: "phase",
    6: "solution",
    7: "prune",
}
UINT64_MAX = (1 << 64) - 1


def _load_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"Expected a JSON object in {path}")
    return value


def _trace_record(trace: PlannerTrace) -> dict[str, Any]:
    if trace.truncated:
        raise RuntimeError("The planner trace exceeded its configured memory limit")
    events: list[list[int | float | None]] = []
    for row in trace.events:
        kind = int(row["kind"])
        if kind not in TRACE_KIND:
            raise ValueError(f"Unknown native trace event kind: {kind}")
        node = int(row["node"])
        parent = int(row["parent"])
        value = float(row["value"])
        events.append(
            [
                -1 if node == UINT64_MAX else node,
                -1 if parent == UINT64_MAX else parent,
                value if math.isfinite(value) else None,
                kind,
                int(row["side"]),
            ]
        )
    points = None if trace.points is None else trace.points.astype(float).tolist()
    return {
        "kind": trace.kind,
        "event_columns": ["node_id", "parent_id", "value", "kind", "side"],
        "event_kinds": TRACE_KIND,
        "events": events,
        "points": points,
    }


def _expanded_count(trace: PlannerTrace) -> int:
    return int(np.count_nonzero(trace.events["kind"] == 2))


def _result_path(result: Any) -> np.ndarray:
    if not result.success or result.path is None:
        raise RuntimeError(f"Planner did not return a path: {result.stop_reason}")
    path = np.asarray(result.path)
    if path.ndim == 0 or not len(path):
        raise RuntimeError("Planner returned an empty path")
    return path


def _cost_matches(actual: float, expected: float, *, tolerance: float = 1e-9) -> bool:
    return math.isfinite(actual) and abs(actual - expected) <= tolerance


def _load_map_rows(path: Path) -> list[str]:
    lines = path.read_text(encoding="ascii").splitlines()
    marker = lines.index("map")
    rows = lines[marker + 1 :]
    if not rows or any(not row for row in rows):
        raise ValueError(f"Invalid MovingAI ASCII map: {path}")
    return rows


def _run_astar() -> dict[str, Any]:
    case = _load_json(INPUTS / "movingai_ost102d.json")
    map_path = INPUTS / "ost102d.map"
    map_hash = hashlib.sha256(map_path.read_bytes()).hexdigest()
    if map_hash != case["source"]["sha256"]:
        raise ValueError("The bundled MovingAI map does not match its recorded SHA-256")

    grid = load_map(map_path)
    start = tuple(case["start"])
    goal = tuple(case["goal"])
    seed = int(case["seed"])
    result = plan_discrete(
        DiscreteProblem(grid, grid.id_for(start), grid.id_for(goal)),
        planner="astar",
        seed=seed,
        trace=TraceOptions(max_bytes=TRACE_LIMIT_BYTES),
    )
    if result.trace is None:
        raise RuntimeError("A* did not return the requested native trace")

    path_ids = _result_path(result).astype(np.int64, copy=False).reshape(-1)
    path_cells = [list(grid.cell_for(int(node_id))) for node_id in path_ids]
    path_cost = sum(
        grid.edge_cost(int(path_ids[index]), int(path_ids[index + 1]))
        for index in range(len(path_ids) - 1)
    )
    valid = (
        path_cells[0] == list(start)
        and path_cells[-1] == list(goal)
        and all(grid.is_passable(int(node_id)) for node_id in path_ids)
        and all(
            math.isfinite(grid.edge_cost(int(path_ids[index]), int(path_ids[index + 1])))
            for index in range(len(path_ids) - 1)
        )
        and _cost_matches(float(result.stats["path_cost"]), path_cost)
    )
    if not valid:
        raise RuntimeError("A* path failed the independent map and cost validation")

    return {
        "id": "astar-movingai-ost102d-s57",
        "planner": "A*",
        "planner_id": "astar",
        "family": "discrete",
        "title": "A* on MovingAI DAO / ost102d",
        "summary": "An 8-neighbor MovingAI scenario replayed from the package's native A* trace.",
        "seed": {
            "value": seed,
            "meaning": "API seed; A* is deterministic and draws no random samples.",
        },
        "input": {
            "environment": "MovingAI DAO / ost102d.map",
            "source": case["source"],
            "width": grid.width,
            "height": grid.height,
            "map_rows": _load_map_rows(map_path),
            "start": list(start),
            "goal": list(goal),
            "movement": case["movement"],
        },
        "parameters": {"planner": "astar", "seed": seed},
        "result": {
            "success": bool(result.success),
            "stop_reason": str(result.stop_reason.value),
            "path": path_cells,
            "path_cost": float(result.stats["path_cost"]),
            "expanded_nodes": _expanded_count(result.trace),
            "discovered_nodes": int(result.nodes),
            "iterations": int(result.iters),
            "valid": True,
            "validity_method": "Every cell and 8-neighbor edge checked against the original MovingAI graph; cost recomputed from edge weights.",
            "published_scenario_cost": float(case["source"]["published_optimal_cost"]),
        },
        "phases": [
            {
                "id": "search",
                "label": "A* search",
                "path": path_cells,
                "path_cost": float(result.stats["path_cost"]),
                "expanded_nodes": _expanded_count(result.trace),
                "trace": _trace_record(result.trace),
                "valid": True,
                "validity_method": "Every cell and 8-neighbor edge checked against the original MovingAI graph; cost recomputed from edge weights.",
            }
        ],
    }


def _grid_path_valid(
    space: Grid2DSearchSpace,
    path: Sequence[Sequence[int]],
    start: Sequence[int],
    goal: Sequence[int],
) -> tuple[bool, float]:
    cells = [tuple(map(int, cell)) for cell in path]
    valid = (
        bool(cells)
        and cells[0] == tuple(start)
        and cells[-1] == tuple(goal)
        and all(space.is_valid_node(cell) for cell in cells)
        and all(math.isfinite(space.edge_cost(a, b)) for a, b in zip(cells, cells[1:]))
    )
    cost = sum(space.edge_cost(a, b) for a, b in zip(cells, cells[1:]))
    return valid, float(cost)


def _run_dstar() -> dict[str, Any]:
    case = _load_json(INPUTS / "dstar_lite_obstacle_insertion.json")
    width, height = int(case["width"]), int(case["height"])
    wall_x = int(case["wall_x"])
    openings = set(map(int, case["openings_before_update"]))
    blocked = {(wall_x, y) for y in range(height) if y not in openings}
    start = tuple(map(int, case["start"]))
    goal = tuple(map(int, case["goal"]))
    space = Grid2DSearchSpace(
        width=width,
        height=height,
        motions=tuple(tuple(motion) for motion in case["motions"]),
        obstacles=blocked,
    )
    planner = DStarLitePlanner(space, start, goal)
    trace_options = TraceOptions(max_bytes=TRACE_LIMIT_BYTES)
    initial = planner.plan(trace=trace_options)
    if initial.trace is None:
        raise RuntimeError("D* Lite did not return the requested initial trace")
    initial_cells = np.asarray(_result_path(initial)).astype(int).tolist()
    initial_valid, initial_cost = _grid_path_valid(space, initial_cells, start, goal)
    if not initial_valid or not _cost_matches(float(initial.stats["path_cost"]), initial_cost):
        raise RuntimeError("Initial D* Lite path failed independent grid validation")

    inserted = tuple(map(int, case["inserted_obstacle"]))
    next_blocked = blocked | {inserted}
    neighbors = ((inserted[0] - 1, inserted[1]), (inserted[0] + 1, inserted[1]))
    updates = [(inserted, neighbor, math.inf) for neighbor in neighbors] + [
        (neighbor, inserted, math.inf) for neighbor in neighbors
    ]
    # Reuse the same D* Lite session after the map and its four directed edges change.
    space.update_obs(next_blocked)
    planner.update_edges(updates)
    repaired = planner.plan(trace=trace_options)
    if repaired.trace is None:
        raise RuntimeError("D* Lite did not return the requested repair trace")
    repaired_cells = np.asarray(_result_path(repaired)).astype(int).tolist()
    repaired_valid, repaired_cost = _grid_path_valid(space, repaired_cells, start, goal)
    if not repaired_valid or not _cost_matches(float(repaired.stats["path_cost"]), repaired_cost):
        raise RuntimeError("Repaired D* Lite path failed independent grid validation")
    if inserted in map(tuple, repaired_cells):
        raise RuntimeError("Repaired D* Lite path still passes through the inserted obstacle")

    def make_phase(
        phase_id: str,
        label: str,
        path: list[list[int]],
        cost: float,
        trace: PlannerTrace,
        validity_method: str,
    ) -> dict[str, Any]:
        return {
            "id": phase_id,
            "label": label,
            "path": path,
            "path_cost": cost,
            "expanded_nodes": _expanded_count(trace),
            "trace": _trace_record(trace),
            "valid": True,
            "validity_method": validity_method,
        }

    before_rows = [
        "".join("#" if (x, y) in blocked else "." for x in range(width)) for y in range(height)
    ]
    after_rows = [
        "".join("#" if (x, y) in next_blocked else "." for x in range(width)) for y in range(height)
    ]
    return {
        "id": "dstar-lite-obstacle-insertion-grid-24x16",
        "planner": "D* Lite",
        "planner_id": "dstar_lite",
        "family": "discrete",
        "title": "D* Lite repairs after an obstacle appears",
        "summary": "One persistent D* Lite session updates four directed edge costs and repairs its route.",
        "seed": {
            "value": None,
            "meaning": "Not applicable; deterministic graph search uses no random samples.",
        },
        "input": {
            "environment": "Defined 24 x 16 four-connected demonstration grid; not an external benchmark.",
            "source": case["source"],
            "width": width,
            "height": height,
            "start": list(start),
            "goal": list(goal),
            "movement": "Four cardinal neighbors; unit edge cost.",
            "before_rows": before_rows,
            "after_rows": after_rows,
            "inserted_obstacle": list(inserted),
            "directed_edges_updated": len(updates),
        },
        "parameters": {"planner": "DStarLitePlanner", "use_heuristic": True},
        "result": {
            "success": bool(repaired.success),
            "stop_reason": repaired.stop_reason.value,
            "path": repaired_cells,
            "path_cost": float(repaired.stats["path_cost"]),
            "expanded_nodes": _expanded_count(repaired.trace),
            "valid": True,
            "validity_method": "Each cell and cardinal edge checked against the updated graph; the inserted cell is absent and path cost is recomputed.",
            "initial_path_cost": initial_cost,
            "repair_path_cost": repaired_cost,
            "same_planner_instance_reused": True,
        },
        "phases": [
            make_phase(
                "initial-plan",
                "Initial plan",
                initial_cells,
                initial_cost,
                initial.trace,
                "Each cell, cardinal edge, endpoint, and cost checked against the original grid.",
            ),
            make_phase(
                "repair",
                "Repair after obstacle insertion",
                repaired_cells,
                repaired_cost,
                repaired.trace,
                "Each cell and cardinal edge checked against the updated grid; the inserted cell is absent and cost is recomputed.",
            ),
        ],
    }


def _point_is_free(point: Sequence[float], circles: Sequence[Sequence[float]]) -> bool:
    px, py = map(float, point)
    return all((px - cx) ** 2 + (py - cy) ** 2 > radius**2 for cx, cy, radius in circles)


def _segment_is_free(
    start: Sequence[float],
    goal: Sequence[float],
    circles: Sequence[Sequence[float]],
) -> bool:
    ax, ay = map(float, start)
    bx, by = map(float, goal)
    dx, dy = bx - ax, by - ay
    denominator = dx * dx + dy * dy
    for cx, cy, radius in circles:
        if denominator == 0.0:
            nearest_x, nearest_y = ax, ay
        else:
            amount = max(0.0, min(1.0, ((cx - ax) * dx + (cy - ay) * dy) / denominator))
            nearest_x, nearest_y = ax + amount * dx, ay + amount * dy
        if (cx - nearest_x) ** 2 + (cy - nearest_y) ** 2 <= radius**2:
            return False
    return True


def _run_rrt_star() -> dict[str, Any]:
    case = _load_json(INPUTS / "barn_world_236.json")
    source = case["source"]
    circles = [[float(value) for value in row] for row in case["source_circles"]]
    bounds = np.asarray(case["bounds"], dtype=np.float64)
    margin = float(case["planning"]["obstacle_radius_inflation"])
    planning_circles = [[cx, cy, radius + margin] for cx, cy, radius in circles]
    start = np.asarray(case["query"]["start"], dtype=np.float64)
    goal = np.asarray(case["query"]["goal"], dtype=np.float64)
    seed = int(case["query"]["seed"])
    collision_step = float(case["planning"]["collision_step"])

    space = Grid2DSamplingSpace(
        x_range=(float(bounds[0, 0]), float(bounds[1, 0])),
        y_range=(float(bounds[0, 1]), float(bounds[1, 1])),
        obs_circle=planning_circles,
        delta=0.0,
        collision_step=collision_step,
        max_sample_tries=int(case["planning"]["max_sample_tries"]),
    )
    params = RrtParams(
        max_iters=int(case["planning"]["max_iters"]),
        step_size=float(case["planning"]["step_size"]),
        goal_sample_rate=float(case["planning"]["goal_sample_rate"]),
        time_budget_s=None,
        max_sample_tries=int(case["planning"]["max_sample_tries"]),
        collision_step=collision_step,
        allow_python_callbacks=bool(case["planning"]["allow_python_callbacks"]),
    )
    result = plan_continuous(
        ContinuousProblem(space=space, start=start, goal=GoalState(state=goal)),
        planner="rrt_star",
        params=params,
        seed=seed,
        trace=TraceOptions(max_bytes=TRACE_LIMIT_BYTES),
    )
    if result.trace is None or result.trace.points is None:
        raise RuntimeError("RRT* did not return the requested points and native trace")
    path = _result_path(result).astype(np.float64, copy=False)
    source_geometry_valid = (
        all(
            bounds[0, axis] <= point[axis] <= bounds[1, axis] for point in path for axis in range(2)
        )
        and all(_point_is_free(point, circles) for point in path)
        and all(
            _segment_is_free(path[index], path[index + 1], circles)
            for index in range(len(path) - 1)
        )
    )
    recomputed_cost = float(
        sum(np.linalg.norm(path[index + 1] - path[index]) for index in range(len(path) - 1))
    )
    if not source_geometry_valid or not _cost_matches(
        float(result.stats["path_cost"]), recomputed_cost
    ):
        raise RuntimeError("RRT* path failed independent source-geometry or cost validation")

    return {
        "id": "rrt-star-barn-world236-point-xy-seed-6294006",
        "planner": "RRT*",
        "planner_id": "rrt_star",
        "family": "continuous",
        "title": "RRT* in BARN world_236 geometry",
        "summary": "A native RRT* run on a point-robot XY query derived from BARN world_236.",
        "seed": {
            "value": seed,
            "meaning": "NumPy default_rng seed passed through the package API.",
        },
        "input": {
            "environment": "BARN world_236, source-derived cylinder geometry.",
            "source": source,
            "robot_model": case["robot_model"],
            "bounds": bounds.tolist(),
            "source_obstacle_count": len(circles),
            "source_circles": circles,
            "planning_obstacle_radius_inflation": margin,
            "start": start.tolist(),
            "goal": goal.tolist(),
            "query": case["query"],
            "movement": "Euclidean straight-line local connections; collision checking at 0.01 m.",
        },
        "parameters": {
            "planner": "rrt_star",
            "max_iters": params.max_iters,
            "step_size": params.step_size,
            "goal_sample_rate": params.goal_sample_rate,
            "time_budget_s": params.time_budget_s,
            "max_sample_tries": params.max_sample_tries,
            "collision_step": params.collision_step,
            "rrt_star_radius_gamma": params.rrt_star_radius_gamma,
            "rrt_star_radius_bias": params.rrt_star_radius_bias,
            "rrt_star_radius_max_factor": params.rrt_star_radius_max_factor,
            "allow_python_callbacks": params.allow_python_callbacks,
        },
        "result": {
            "success": bool(result.success),
            "stop_reason": result.stop_reason.value,
            "path": path.tolist(),
            "path_cost": float(result.stats["path_cost"]),
            "iterations": int(result.iters),
            "tree_vertices": int(result.nodes),
            "samples_recorded": int(np.count_nonzero(result.trace.events["kind"] == 4)),
            "expanded_nodes": None,
            "valid": True,
            "validity_method": "All path states and complete line segments checked analytically against all 254 original source circles; path length recomputed in Euclidean XY.",
        },
        "phases": [
            {
                "id": "sampling-and-rewiring",
                "label": "Sampling and rewiring",
                "path": path.tolist(),
                "path_cost": float(result.stats["path_cost"]),
                "iterations": int(result.iters),
                "tree_vertices": int(result.nodes),
                "trace": _trace_record(result.trace),
                "valid": True,
                "validity_method": "All path states and complete line segments checked analytically against all original source circles; path length recomputed in Euclidean XY.",
            }
        ],
    }


def build_runs() -> dict[str, Any]:
    package_version = version("pathplanning")
    runs = [
        _run_astar(),
        _run_dstar(),
        _run_rrt_star(),
    ]
    return {
        "schema_version": 1,
        "package": {"name": "pathplanning", "version": package_version},
        "runtime": {"python_version": ".".join(map(str, sys.version_info[:3]))},
        "trace_contract": {
            "origin": "Native package trace captured while each planner executed.",
            "event_columns": ["node_id", "parent_id", "value", "kind", "side"],
            "sentinel": -1,
            "null_value_means": "Non-finite planner value, usually positive infinity.",
            "event_kinds": TRACE_KIND,
        },
        "reproduction_command": ".venv/bin/python -m scripts.generate_planner_demo_runs",
        "runs": runs,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    payload = build_runs()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(payload, ensure_ascii=True, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    print(f"Wrote {len(payload['runs'])} package-backed planner runs to {args.output}")
    for run in payload["runs"]:
        result = run["result"]
        work = result.get("expanded_nodes")
        if work is None:
            work = f"{result.get('iterations')} iterations / {result.get('tree_vertices')} tree vertices"
        print(
            f"{run['planner']}: success={result['success']}, cost={result['path_cost']:.6f}, "
            f"work={work}, valid={result['valid']}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
