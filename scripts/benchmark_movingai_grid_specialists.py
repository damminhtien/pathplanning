"""Benchmark grid-specialist planners on stratified MovingAI land workloads."""

from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import sys
import time
from typing import Any

import numpy as np

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from pathplanning import plan_discrete
from pathplanning.core.contracts import DiscreteProblem
from pathplanning.spaces.grid2d import Grid2DSearchSpace
from scripts.shortest_path_benchmark.workloads import MOVEMENT_PROFILE, MovingAIGrid, load_map

_PLANNERS = ("jps", "dstar_lite", "theta_star", "lazy_theta_star")
_DSTAR_NODE_LIMIT = 1_000_000
_ANY_ANGLE_PROFILE = "any_angle_square_obstacle_visibility_v1"


def _stable_id(value: Any) -> str:
    encoded = json.dumps(value, sort_keys=True, separators=(",", ":")).encode()
    return hashlib.sha256(encoded).hexdigest()


def _read_jsonl(path: Path) -> list[dict[str, Any]]:
    if not path.is_file():
        return []
    content = path.read_bytes()
    complete_end = content.rfind(b"\n") + 1
    if complete_end != len(content):
        with path.open("r+b") as stream:
            stream.truncate(complete_end)
    return [
        json.loads(line)
        for line in content[:complete_end].decode("utf-8").splitlines()
        if line.strip()
    ]


def _grid_path_metrics(grid: MovingAIGrid, path: Any) -> tuple[bool, float | None]:
    values = np.asarray(path)
    if values.ndim == 2 and values.shape[1] == 1:
        values = values[:, 0]
    if values.ndim == 1:
        nodes = [int(value) for value in values]
        points = [(node % grid.width, node // grid.width) for node in nodes]
    elif values.ndim == 2 and values.shape[1] == 2:
        coordinates = np.asarray(values, dtype=np.float64)
        if not np.all(np.isfinite(coordinates)):
            return False, None
        rounded = np.rint(coordinates)
        if not np.allclose(coordinates, rounded, rtol=0.0, atol=1e-8):
            return False, None
        points = [(int(point[0]), int(point[1])) for point in rounded]
    else:
        return False, None
    if len(points) < 2:
        return False, None

    total = 0.0
    for first, last in zip(points, points[1:]):
        dx, dy = last[0] - first[0], last[1] - first[1]
        steps = max(abs(dx), abs(dy))
        if steps == 0 or (dx and dy and abs(dx) != abs(dy)):
            return False, None
        sx = 0 if dx == 0 else (1 if dx > 0 else -1)
        sy = 0 if dy == 0 else (1 if dy > 0 else -1)
        current = first
        for _ in range(steps):
            following = (current[0] + sx, current[1] + sy)
            source_id = current[1] * grid.width + current[0]
            target_id = following[1] * grid.width + following[0]
            cost = grid.edge_cost(source_id, target_id)
            if not math.isfinite(cost):
                return False, None
            total += cost
            current = following
    return True, total


def _segment_visible(
    grid: MovingAIGrid, first: tuple[float, float], last: tuple[float, float]
) -> bool:
    """Supercover traversal; cells touched at a corner count as occupied."""
    x0, y0 = first
    x1, y1 = last
    dx, dy = x1 - x0, y1 - y0
    x, y = math.floor(x0 + 0.5), math.floor(y0 + 0.5)
    end_x, end_y = math.floor(x1 + 0.5), math.floor(y1 + 0.5)
    sx = 0 if dx == 0 else (1 if dx > 0 else -1)
    sy = 0 if dy == 0 else (1 if dy > 0 else -1)
    delta_x = math.inf if dx == 0 else abs(1.0 / dx)
    delta_y = math.inf if dy == 0 else abs(1.0 / dy)
    boundary_x = x + (0.5 if sx > 0 else -0.5)
    boundary_y = y + (0.5 if sy > 0 else -0.5)
    max_x = math.inf if dx == 0 else (boundary_x - x0) / dx
    max_y = math.inf if dy == 0 else (boundary_y - y0) / dy

    def free(cell_x: int, cell_y: int) -> bool:
        return (
            0 <= cell_x < grid.width
            and 0 <= cell_y < grid.height
            and grid.is_passable(cell_y * grid.width + cell_x)
        )

    if not free(x, y) or not free(end_x, end_y):
        return False
    while (x, y) != (end_x, end_y):
        if max_x < max_y - 1e-12:
            x += sx
            max_x += delta_x
            if not free(x, y):
                return False
        elif max_y < max_x - 1e-12:
            y += sy
            max_y += delta_y
            if not free(x, y):
                return False
        else:
            if not free(x + sx, y) or not free(x, y + sy):
                return False
            x += sx
            y += sy
            max_x += delta_x
            max_y += delta_y
            if not free(x, y):
                return False
    return True


def _any_angle_metrics(
    grid: MovingAIGrid, path: Any, start: tuple[int, int], goal: tuple[int, int]
) -> tuple[bool, float | None]:
    points = np.asarray(path, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 2 or len(points) < 2:
        return False, None
    if not np.all(np.isfinite(points)):
        return False, None
    if not np.allclose(points[0], start, rtol=0.0, atol=1e-7):
        return False, None
    if not np.allclose(points[-1], goal, rtol=0.0, atol=1e-7):
        return False, None
    visible = all(
        _segment_visible(grid, tuple(first), tuple(last))
        for first, last in zip(points, points[1:])
    )
    length = float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())
    return visible, length


def _variant_id(planner: str) -> str:
    params = {"max_materialized_nodes": _DSTAR_NODE_LIMIT} if planner == "dstar_lite" else {}
    movement = MOVEMENT_PROFILE if planner in {"jps", "dstar_lite"} else _ANY_ANGLE_PROFILE
    return _stable_id({"algorithm": planner, "parameters": params, "movement_profile": movement})


def _run_one(
    grid: MovingAIGrid,
    workload: dict[str, Any],
    oracle: dict[str, Any],
    planner: str,
    seed: int,
) -> dict[str, Any]:
    start_id, goal_id = int(workload["start"]), int(workload["goal"])
    start, goal = grid.cell_for(start_id), grid.cell_for(goal_id)
    params = {"max_materialized_nodes": _DSTAR_NODE_LIMIT} if planner == "dstar_lite" else {}
    movement = MOVEMENT_PROFILE if planner in {"jps", "dstar_lite"} else _ANY_ANGLE_PROFILE
    record = {
        "schema_version": "pathplanning_movingai_grid_specialist_v1",
        "source": "movingai",
        "dataset_id": workload["map_sha256"],
        "dataset_name": workload["map_path"],
        "cohort": "movingai_land_octile" if movement == MOVEMENT_PROFILE else "movingai_land_any_angle",
        "workload_id": workload["workload_id"],
        "variant_id": _variant_id(planner),
        "planner": planner,
        "parameters": params,
        "seed": seed,
        "input": {
            "map_path": workload["map_path"],
            "map_sha256": workload["map_sha256"],
            "scenario_path": workload["scenario_path"],
            "scenario_line": workload["scenario_line"],
            "start": list(start),
            "goal": list(goal),
            "displacement_bin": workload.get("displacement_bin"),
            "source_scenario_optimum": workload.get("scenario_optimum"),
            "movement_profile": movement,
            "octile_reference_cost": oracle["reference_cost"],
        },
    }
    if planner == "dstar_lite" and grid.node_count > _DSTAR_NODE_LIMIT:
        record.update(
            outcome={"status": "resource_limited", "path_valid": None},
            timing={"public_api_s": None},
            error=f"grid exceeds D* Lite materialization limit {_DSTAR_NODE_LIMIT}",
        )
        return record

    graph = (
        grid
        if planner == "dstar_lite"
        else Grid2DSearchSpace(width=grid.width, height=grid.height, occupancy=grid.occupancy)
    )
    problem = DiscreteProblem(
        graph=graph,
        start=start_id if planner == "dstar_lite" else start,
        goal=goal_id if planner == "dstar_lite" else goal,
    )
    started = time.perf_counter()
    try:
        result = plan_discrete(problem, planner=planner, params=params, seed=seed)
        elapsed = time.perf_counter() - started
        declared_cost = result.stats.get("path_cost")
        if movement == _ANY_ANGLE_PROFILE:
            valid, cost = (
                _any_angle_metrics(grid, result.path, start, goal)
                if result.path is not None
                else (None, None)
            )
            declared_cost_matches = (
                math.isclose(float(declared_cost), cost, rel_tol=1e-10, abs_tol=1e-8)
                if declared_cost is not None and cost is not None
                else None
            )
            if declared_cost_matches is False:
                valid = False
            status = (
                "no_solution_found"
                if not result.success and result.path is None
                else "invalid_path"
                if not valid
                else "valid_any_angle_path"
            )
            optimal = None
        else:
            valid, cost = (
                _grid_path_metrics(grid, result.path)
                if result.path is not None
                else (None, None)
            )
            declared_cost_matches = (
                math.isclose(float(declared_cost), cost, rel_tol=1e-10, abs_tol=1e-8)
                if declared_cost is not None and cost is not None
                else None
            )
            if declared_cost_matches is False:
                valid = False
            if result.path is None:
                status = "no_path"
            elif not valid:
                status = "invalid_path"
            elif math.isclose(cost or 0.0, float(oracle["reference_cost"]), rel_tol=1e-10, abs_tol=1e-8):
                status = "valid_optimal"
            else:
                status = "valid_suboptimal"
            optimal = status == "valid_optimal"
        record.update(
            outcome={
                "status": status,
                "path_valid": valid,
                "path_cost": cost,
                "declared_cost": declared_cost,
                "declared_cost_matches": declared_cost_matches,
                "octile_reference_cost": oracle["reference_cost"],
                "optimal_in_octile_profile": optimal,
                "planner_success": bool(result.success),
                "iters": int(result.iters),
                "nodes": int(result.nodes),
            },
            timing={"public_api_s": elapsed},
            error=None,
        )
    except Exception as exc:
        record.update(
            outcome={"status": "error", "path_valid": None},
            timing={"public_api_s": time.perf_counter() - started},
            error=f"{type(exc).__name__}: {exc}",
        )
    return record


def run_grid_specialists(
    dataset_root: str | Path,
    manifest_path: str | Path,
    output: str | Path,
    *,
    oracle_path: str | Path | None = None,
    planners: tuple[str, ...] = _PLANNERS,
    seed: int = 7,
) -> dict[str, Any]:
    root = Path(dataset_root).resolve()
    manifest_file = Path(manifest_path).resolve()
    manifest = json.loads(manifest_file.read_text())
    if manifest.get("profile") != "coverage":
        raise ValueError("grid-specialist runner requires a coverage manifest")
    oracle_file = (
        Path(oracle_path).resolve()
        if oracle_path is not None
        else manifest_file.with_suffix(".oracle.jsonl")
    )
    oracle_by_id = {row["workload_id"]: row["outcome"] for row in _read_jsonl(oracle_file)}
    latency_ids = set(manifest["cohorts"]["latency"]["workload_ids"])
    workloads = [
        case
        for case in manifest["cases"]
        if case["workload_id"] in latency_ids
        and case["movement_profile"] == MOVEMENT_PROFILE
    ]
    if not workloads:
        raise ValueError("coverage manifest has no supported land-grid workloads")
    missing = [row["workload_id"] for row in workloads if row["workload_id"] not in oracle_by_id]
    if missing:
        raise ValueError(f"validate the coverage manifest first; {len(missing)} oracle rows are missing")
    chosen = tuple(dict.fromkeys(planners))
    if not chosen or set(chosen) - set(_PLANNERS):
        raise ValueError(f"supported planners are {_PLANNERS}")

    campaign = Path(output).resolve()
    campaign.mkdir(parents=True, exist_ok=True)
    runs_path = campaign / "runs.jsonl"
    prior = _read_jsonl(runs_path)
    completed = {
        (row.get("workload_id"), row.get("variant_id"))
        for row in prior
        if row.get("schema_version") == "pathplanning_movingai_grid_specialist_v1"
        and row.get("outcome", {}).get("status") != "error"
    }
    with runs_path.open("a", encoding="utf-8", buffering=1) as stream:
        for workload in sorted(workloads, key=lambda row: row["map_path"]):
            grid = load_map(root / workload["map_path"])
            if grid.map_sha256 != workload["map_sha256"]:
                raise ValueError(f"map hash mismatch: {workload['map_path']}")
            oracle = oracle_by_id[workload["workload_id"]]
            if oracle.get("reference_reachable") is not True:
                raise ValueError(f"source workload is not oracle-reachable: {workload['workload_id']}")
            for planner in chosen:
                key = (workload["workload_id"], _variant_id(planner))
                if key in completed:
                    continue
                row = _run_one(grid, workload, oracle, planner, seed)
                stream.write(json.dumps(row, sort_keys=True, separators=(",", ":")) + "\n")
                stream.flush()
                if row["outcome"]["status"] != "error":
                    completed.add(key)

    rows = _read_jsonl(runs_path)
    statuses = Counter(row["outcome"]["status"] for row in rows)
    by_planner: dict[str, Counter[str]] = {}
    for row in rows:
        by_planner.setdefault(row["planner"], Counter())[row["outcome"]["status"]] += 1
    summary = {
        "schema_version": "pathplanning_movingai_grid_specialist_summary_v1",
        "manifest_sha256": hashlib.sha256(manifest_file.read_bytes()).hexdigest(),
        "oracle_sha256": hashlib.sha256(oracle_file.read_bytes()).hexdigest(),
        "selected_workloads": len(workloads),
        "measurements": len(rows),
        "planner_count": len({row["planner"] for row in rows}),
        "status_counts": dict(sorted(statuses.items())),
        "status_counts_by_planner": {
            name: dict(sorted(counts.items())) for name, counts in sorted(by_planner.items())
        },
        "excluded": {
            "jpsw": "the source terrain cost table is unavailable",
            "temporal_and_multi_agent": "no dynamic or multi-agent source tasks",
            "kinodynamic": "no source robot/control model",
        },
    }
    (campaign / "summary.json").write_text(
        json.dumps(summary, sort_keys=True, indent=2) + "\n", encoding="utf-8"
    )
    report = [
        "# MovingAI grid-specialist cohort",
        "",
        f"- Unique source workloads: {summary['selected_workloads']}",
        f"- Planner observations: {summary['measurements']}",
        f"- Outcome counts: {json.dumps(summary['status_counts'], sort_keys=True)}",
        "",
        "JPS and D* Lite use the independent no-corner-cutting octile oracle. Theta* and Lazy Theta* are collision-checked in a separate any-angle cohort and are not compared with the octile optimum. JPSW is excluded because the source terrain cost table is missing.",
        "",
    ]
    (campaign / "report.md").write_text("\n".join(report), encoding="utf-8")
    return summary


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    run = commands.add_parser("run", help="run specialized algorithms on one query per land map")
    run.add_argument(
        "--dataset-root",
        type=Path,
        default=Path("benchmark-results/datasets/movingai-v2"),
    )
    run.add_argument("--manifest", type=Path, required=True)
    run.add_argument("--oracle", type=Path)
    run.add_argument("--output", type=Path, required=True)
    run.add_argument("--seed", type=int, default=7)
    run.add_argument("--planners", nargs="+", choices=_PLANNERS, default=_PLANNERS)
    args = parser.parse_args(argv)
    summary = run_grid_specialists(
        args.dataset_root,
        args.manifest,
        args.output,
        oracle_path=args.oracle,
        planners=tuple(args.planners),
        seed=args.seed,
    )
    print(json.dumps(summary, sort_keys=True))
    return int(summary["status_counts"].get("error", 0) > 0)


if __name__ == "__main__":
    raise SystemExit(main())
