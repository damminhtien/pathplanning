"""Prepare, validate, run, and analyze deterministic scaling campaigns."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from scripts.shortest_path_benchmark.analysis import analyze_campaign
from scripts.shortest_path_benchmark.runner import run_campaign, validate_manifest
from scripts.shortest_path_benchmark.scaling_manifest import prepare_scaling_manifest

_DEFAULT_PROFILE = Path(__file__).parent / "shortest_path_benchmark" / "profiles" / "scaling.json"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)

    prepare = commands.add_parser("prepare", help="freeze generated maps and workload ids")
    prepare.add_argument("--profile", type=Path, default=_DEFAULT_PROFILE)
    prepare.add_argument("--manifest", type=Path, required=True)

    validate = commands.add_parser("validate", help="compute independent Dijkstra references")
    validate.add_argument("--manifest", type=Path, required=True)

    run = commands.add_parser("run", help="run one separated measurement pass")
    run.add_argument("--manifest", type=Path, required=True)
    run.add_argument("--campaign", type=Path, required=True)
    run.add_argument("--pass", dest="pass_name", choices=("latency", "work", "memory"), required=True)
    run.add_argument("--scope", choices=("public_api", "prepared_kernel"), default="public_api")
    run.add_argument("--graph-state", choices=("reused_graph", "fresh_graph"), default="reused_graph")
    run.add_argument("--cohort", help="select one manifest cohort, such as size or topology")
    run.add_argument("--repeats", type=int)
    run.add_argument("--warmups", type=int)
    run.add_argument("--schedule-seed", type=int, default=7)
    run.add_argument("--query-timeout-s", type=float, default=5.0)
    run.add_argument("--setup-timeout-s", type=float, default=60.0)

    analyze = commands.add_parser("analyze", help="write the campaign report")
    analyze.add_argument("--campaign", type=Path, required=True)
    analyze.add_argument("--baseline", default="dijkstra")

    args = parser.parse_args(argv)
    if args.command == "prepare":
        result = prepare_scaling_manifest(args.profile, args.manifest)
        print(
            f"prepared {result['selection']['unique_workloads']} workloads across "
            f"{result['selection']['map_count']} maps in {args.manifest}"
        )
        return 0
    if args.command == "validate":
        result = validate_manifest(args.manifest)
        print(f"checked {result['checked']} workloads; discrepancies={len(result['discrepancies'])}")
        return 1 if result["discrepancies"] else 0
    if args.command == "run":
        result = run_campaign(
            args.manifest,
            args.campaign,
            pass_name=args.pass_name,
            scope=args.scope,
            graph_state=args.graph_state,
            repeats=args.repeats,
            warmups=args.warmups,
            schedule_seed=args.schedule_seed,
            query_timeout_s=args.query_timeout_s,
            setup_timeout_s=args.setup_timeout_s,
            cohort_name=args.cohort,
        )
        print(result)
        return 0 if result["complete"] else 1
    analyze_campaign(args.campaign, baseline=args.baseline)
    print(f"wrote report to {args.campaign / 'report.md'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
