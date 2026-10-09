"""Inventory, characterize and stratify public path-planning benchmark inputs."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from scripts.benchmark_datasets.catalog import discover
from scripts.benchmark_datasets.profiles import Limits
from scripts.benchmark_datasets.report import render_report, stratify_queries
from scripts.benchmark_datasets.runner import run_profiles, write_json


def main(argv: list[str] | None = None) -> int:
    """Run discovery separately from cached, bounded and resumable profiling."""
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    inventory = commands.add_parser("catalog")
    inventory.add_argument("--output", type=Path, required=True)
    profile = commands.add_parser("profile")
    profile.add_argument("--catalog", type=Path, required=True)
    profile.add_argument("--output", type=Path, required=True)
    profile.add_argument("--per-family", type=int, default=3, help="0 profiles every catalog entry")
    profile.add_argument("--max-cells", type=int, default=8_000_000)
    profile.add_argument("--max-arcs", type=int, default=4_000_000)
    profile.add_argument("--max-download-mib", type=int, default=256)
    profile.add_argument("--sample-size", type=int, default=4096)
    profile.add_argument("--seed", type=int, default=7)
    profile.add_argument("--cache", type=Path)
    profile.add_argument("--dataset-root", type=Path)
    profile.add_argument("--dataset-index", type=Path)
    report = commands.add_parser("report")
    report.add_argument("--profiles", type=Path, required=True)
    report.add_argument("--output", type=Path, required=True)
    cohort = commands.add_parser("stratify")
    cohort.add_argument("--queries", type=Path, required=True)
    cohort.add_argument("--output", type=Path, required=True)
    cohort.add_argument("--per-bin", type=int, default=20)
    cohort.add_argument("--seed", type=int, default=7)
    args = parser.parse_args(argv)
    if args.command == "catalog":
        catalog = discover()
        write_json(args.output, catalog)
        print(
            f"Cataloged {len(catalog['entries'])} assets; "
            f"discovery failures: {len(catalog['discovery_failures'])}"
        )
        return 1 if catalog["discovery_failures"] else 0
    if args.command == "profile":
        if args.max_download_mib < 1:
            parser.error("--max-download-mib must be positive")
        if (args.dataset_root is None) != (args.dataset_index is None):
            parser.error("--dataset-root and --dataset-index must be provided together")
        limits = Limits(
            max_cells=args.max_cells,
            max_arcs=args.max_arcs,
            sample_size=args.sample_size,
            seed=args.seed,
        )
        result = run_profiles(
            json.loads(args.catalog.read_text()),
            args.output,
            limits,
            per_family=args.per_family,
            maximum_bytes=args.max_download_mib << 20,
            cache=args.cache,
            dataset_root=args.dataset_root,
            dataset_index=args.dataset_index,
        )
        render_report(result, args.output)
        return 1 if any(record["status"] == "error" for record in result["records"]) else 0
    if args.command == "stratify":
        write_json(
            args.output,
            stratify_queries(
                json.loads(args.queries.read_text()), per_bin=args.per_bin, seed=args.seed
            ),
        )
        return 0
    render_report(json.loads(args.profiles.read_text()), args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
