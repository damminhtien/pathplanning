"""Install, inspect and verify the repository's public benchmark datasets."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from typing import Any

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from scripts.benchmark_datasets.installer import (
    DEFAULT_WORKERS,
    SOURCES,
    InstallError,
    install_catalog,
    load_catalog,
    status_report,
    verify_installation,
)


def _human_bytes(value: int) -> str:
    amount = float(value)
    for unit in ("B", "KiB", "MiB", "GiB", "TiB"):
        if amount < 1024 or unit == "TiB":
            return f"{amount:.1f} {unit}"
        amount /= 1024
    raise AssertionError("unreachable")


def _print_status(report: dict[str, Any]) -> None:
    print(f"Catalog entries: {report['catalog_entries']}")
    print("Source       Assets    Status")
    for source, details in report["by_source"].items():
        counts = (
            ", ".join(f"{name} {count}" for name, count in details.items() if name != "catalog")
            or "not installed"
        )
        print(f"{source:<12} {details['catalog']:>5}    {counts}")
    print(f"Files: {report['assets']}")
    if report["discovery_failures"]:
        print(f"Discovery failures: {report['discovery_failures']}")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    install = commands.add_parser("install", help="download and register dataset files")
    install.add_argument("--root", type=Path, default=Path("benchmark-results/datasets"))
    install.add_argument("--catalog", type=Path)
    install.add_argument("--sources", nargs="+", choices=SOURCES, default=list(SOURCES))
    install.add_argument("--workers", type=int, default=DEFAULT_WORKERS)
    install.add_argument("--max-asset-gib", type=float, default=8.0)
    install.add_argument("--max-expanded-gib", type=float, default=16.0)
    install.add_argument("--min-free-gib", type=float, default=2.0)
    status = commands.add_parser("status", help="summarize installed, missing and failed inputs")
    status.add_argument("--root", type=Path, default=Path("benchmark-results/datasets"))
    status.add_argument("--catalog", type=Path)
    verify = commands.add_parser("verify", help="check hashes and source-file references")
    verify.add_argument("--root", type=Path, default=Path("benchmark-results/datasets"))
    verify.add_argument("--sources", nargs="+", choices=SOURCES, default=list(SOURCES))
    args = parser.parse_args(argv)

    try:
        if args.command == "install":
            limits = (
                int(args.max_asset_gib * (1 << 30)),
                int(args.max_expanded_gib * (1 << 30)),
                int(args.min_free_gib * (1 << 30)),
            )
            if min(limits) < 1:
                parser.error("size and free-space limits must be positive")
            catalog = load_catalog(args.catalog, args.root)
            index = install_catalog(
                catalog,
                args.root,
                sources=set(args.sources),
                workers=args.workers,
                maximum_asset_bytes=limits[0],
                maximum_expanded_bytes=limits[1],
                minimum_free_bytes=limits[2],
            )
            _print_status(status_report(args.root, catalog))
            issues = [
                row
                for row in index["datasets"].values()
                if row["source"] in args.sources and row["status"] in {"error", "pending"}
            ]
            source_gaps = [
                row
                for row in index["datasets"].values()
                if row["source"] in args.sources and row["status"] == "missing_source_asset"
            ]
            if source_gaps:
                print(f"Datasets with assets absent from source: {len(source_gaps)}")
            result = verify_installation(args.root, sources=set(args.sources))
            print(
                f"Verified files: {result['checked_assets']}; warnings: {len(result['warnings'])}"
            )
            if result["errors"]:
                for error in result["errors"][:20]:
                    print(f"ERROR: {error}")
            return 1 if issues or result["errors"] or catalog.get("discovery_failures") else 0

        if args.command == "status":
            catalog = load_catalog(args.catalog, args.root)
            _print_status(status_report(args.root, catalog))
            return 1 if catalog.get("discovery_failures") else 0

        result = verify_installation(args.root, sources=set(args.sources))
        print(f"Verified files: {result['checked_assets']}")
        for warning in result["warnings"]:
            print(f"WARNING: {warning}")
        for error in result["errors"]:
            print(f"ERROR: {error}")
        return 0 if result["valid"] else 1
    except (InstallError, OSError, ValueError, json.JSONDecodeError) as exc:
        parser.exit(1, f"dataset installer: {exc}\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
