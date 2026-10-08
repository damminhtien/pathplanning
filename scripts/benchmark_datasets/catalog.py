"""Discover official single-agent catalogs and preserve source provenance.

Discovery is separate from measurement. An inaccessible collection remains an
explicit catalog failure; it must never disappear from the coverage denominator.
"""

from __future__ import annotations

from collections.abc import Callable
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
import hashlib
from html.parser import HTMLParser
import json
from pathlib import Path
import re
from typing import Any
from urllib.parse import urljoin
from urllib.request import Request, urlopen

SCHEMA = "pathplanning_dataset_characterization_v1"
MOVINGAI = "https://www.movingai.com/benchmarks/"
DIMACS = "https://www.diag.uniroma1.it/challenge9/"
BITBUCKET = "https://api.bitbucket.org/2.0/repositories/shortestpathlab/benchmarks/"
FAMILIES = (
    "dao",
    "da2",
    "wc3maps512",
    "bg512",
    "bgmaps",
    "sc1",
    "street",
    "maze",
    "random",
    "room",
    "weighted",
    "warframe",
)


def fetch(url: str, *, maximum_bytes: int = 256 << 20) -> bytes:
    """Read a bounded public asset with a finite network timeout."""
    request = Request(url, headers={"User-Agent": "pathplanning-dataset-characterization/1"})
    with urlopen(request, timeout=30) as response:
        length = response.headers.get("Content-Length")
        if length and int(length) > maximum_bytes:
            raise ValueError("download_limit")
        data = response.read(maximum_bytes + 1)
    if len(data) > maximum_bytes:
        raise ValueError("download_limit")
    return data


def cached_fetch(url: str, cache: Path, *, maximum_bytes: int = 256 << 20) -> Path:
    """Reuse immutable content-addressed URL cache; callers verify content hashes."""
    name = hashlib.sha256(url.encode()).hexdigest() + "-" + url.rsplit("/", 1)[-1].split("?")[0]
    path = cache / name
    if path.exists():
        if path.stat().st_size > maximum_bytes:
            raise ValueError("download_limit")
        return path
    cache.mkdir(parents=True, exist_ok=True)
    data = fetch(url, maximum_bytes=maximum_bytes)
    temp = path.with_suffix(path.suffix + ".part")
    temp.write_bytes(data)
    temp.replace(path)
    return path


class TableParser(HTMLParser):
    """Collect table rows and links without depending on page presentation."""

    def __init__(self) -> None:
        super().__init__()
        self.rows: list[dict[str, Any]] = []
        self.links: list[str] = []
        self.row: dict[str, Any] | None = None

    def handle_starttag(self, tag: str, attrs: list[tuple[str, str | None]]) -> None:
        if tag == "tr":
            self.row = {"text": [], "links": []}
        if tag == "a":
            href = dict(attrs).get("href")
            if href:
                self.links.append(href)
                if self.row is not None:
                    self.row["links"].append(href)

    def handle_endtag(self, tag: str) -> None:
        if tag == "tr" and self.row is not None:
            self.rows.append(self.row)
            self.row = None

    def handle_data(self, data: str) -> None:
        if self.row is not None and data.strip():
            self.row["text"].append(data.strip())


def _parse(url: str) -> TableParser:
    parser = TableParser()
    parser.feed(fetch(url, maximum_bytes=8 << 20).decode("utf-8"))
    return parser


def _entry(
    source: str, family: str, name: str, url: str, representation: str, **metadata: Any
) -> dict[str, Any]:
    identifier = hashlib.sha256(f"{source}|{family}|{name}|{url}".encode()).hexdigest()
    return {
        "dataset_id": identifier,
        "source": source,
        "family": family,
        "name": name,
        "url": url,
        "representation": representation,
        "static": True,
        "single_agent": True,
        "metadata": metadata,
    }


def movingai_catalog(family: str) -> list[dict[str, Any]]:
    """All maps in a collection, keeping original and scaled families distinct."""
    url = urljoin(MOVINGAI, f"{family}/index.html")
    parser = _parse(url)
    result = []
    for row in parser.rows:
        maps = [link for link in row["links"] if re.search(r"\.(?:3dmap|map)(?:\.zip)?$", link)]
        scenarios = [
            link
            for link in row["links"]
            if re.search(r"(?:\.(?:3dscen|scen)|-scen)(?:\.zip)?$", link)
        ]
        for link in maps:
            name = link.removesuffix(".zip")
            text = " ".join(row["text"])
            dimensions = re.search(r"(\d+)\s*[xX]\s*(\d+)(?:\s*[xX]\s*(\d+))?", text)
            shape = [int(value) for value in dimensions.groups() if value] if dimensions else None
            representation = (
                "voxel3d"
                if "3dmap" in name
                else ("terrain2d" if family == "weighted" else "grid2d")
            )
            lineage_family = "baldurs_gate" if family in {"bg512", "bgmaps"} else family
            stem = re.sub(r"\.map$|\.3dmap$", "", name)
            result.append(
                _entry(
                    "movingai",
                    family,
                    name,
                    urljoin(url, link),
                    representation,
                    dimensions_claim=shape,
                    catalog_url=url,
                    scenario_url=urljoin(url, scenarios[0]) if scenarios else None,
                    lineage=f"movingai:{lineage_family}:{stem}",
                    version="corrected_2019" if family == "warframe" else "v2",
                    directed=False,
                    cost_model=(
                        "missing_terrain_table"
                        if representation == "terrain2d"
                        else "euclidean_grid"
                    ),
                )
            )
    if not result:
        raise ValueError(f"Empty map collection: {family}")
    return result


def dimacs_catalog() -> list[dict[str, Any]]:
    """Core distance/time road graphs and Rome99; contributed sets stay documented."""
    url = urljoin(DIMACS, "download.shtml")
    parser = _parse(url)
    result = []
    for link in parser.links:
        if re.search(r"USA-road-[dt]\.[A-Z]+\.gr\.gz$", link) or link.endswith("rome99.gr"):
            name = link.rsplit("/", 1)[-1]
            if name.startswith("USA-road"):
                region = name.split(".")[1]
                mode = "travel_time" if "USA-road-t" in name else "distance"
                lineage = f"dimacs:tiger:{region}"
            else:
                region, mode, lineage = "rome99", "source_weight", "dimacs:rome99"
            result.append(
                _entry(
                    "dimacs",
                    mode,
                    name,
                    urljoin(DIMACS, link),
                    "directed_graph",
                    directed=True,
                    cost_model=mode,
                    region=region,
                    lineage=lineage,
                    dependency_group="tiger_usa" if region != "rome99" else "rome99",
                    catalog_url=url,
                )
            )
    if len(result) != 25:
        raise ValueError(f"DIMACS core inventory changed: expected 25 graphs, found {len(result)}")
    return result


def github_tree(repo: str) -> tuple[str, list[dict[str, Any]]]:
    """Resolve the moving branch once and pin every raw-file URL to that commit."""
    data = json.loads(fetch(f"https://api.github.com/repos/{repo}/git/trees/main?recursive=1"))
    if data.get("truncated"):
        raise ValueError(f"Truncated GitHub tree: {repo}")
    return data["sha"], data["tree"]


def barn_catalog() -> list[dict[str, Any]]:
    """Inventory all static BARN worlds from the challenge's public mirror."""
    repo = "Daffan/the-barn-challenge"
    revision, tree = github_tree(repo)
    result = []
    for item in tree:
        path = item["path"]
        if re.fullmatch(r"jackal_helper/worlds/BARN/world_\d+\.world", path):
            index = path.rsplit("/", 1)[-1].removeprefix("world_").removesuffix(".world")
            result.append(
                _entry(
                    "barn",
                    "static_cylinders",
                    path.rsplit("/", 1)[-1],
                    f"https://raw.githubusercontent.com/{repo}/{revision}/{path}",
                    "continuous2d",
                    revision=revision,
                    lineage=f"barn:{path}",
                    directed=False,
                    cost_model="euclidean_length",
                    catalog_url="https://www.cs.utexas.edu/~xiao/BARN/BARN.html",
                    format="sdf",
                    robot_model="unspecified_source_footprint",
                    path_url=f"https://raw.githubusercontent.com/{repo}/{revision}/"
                    f"jackal_helper/worlds/BARN/path_files/path_{index}.npy",
                )
            )
    if len(result) != 300:
        raise ValueError(f"BARN static inventory changed: expected 300, found {len(result)}")
    return result


def bitbucket_files(path: str, revision: str = "master") -> list[dict[str, Any]]:
    """Follow pagination; do not silently omit the tail of a large collection."""
    url = f"{BITBUCKET}src/{revision}/{path}?pagelen=100"
    result = []
    while url:
        data = json.loads(fetch(url))
        result.extend(data["values"])
        url = data.get("next")
    return result


def monash_catalog() -> list[dict[str, Any]]:
    """Pin Monash's three new voxel collections and its older Warframe mirror."""
    branch = json.loads(fetch(f"{BITBUCKET}refs/branches/master"))
    revision = branch["target"]["hash"]
    result = []
    for family in ("descent", "industrial-plants", "sandstone", "warframe"):
        files = bitbucket_files(f"voxel-maps/{family}/map_files/", revision)
        scenarios = bitbucket_files(f"voxel-maps/{family}/scen_files/", revision)
        scenario_urls = {
            re.sub(r"\.3dscen(?:\.zip)?$", "", item["path"].rsplit("/", 1)[-1]): item["links"][
                "self"
            ]["href"]
            for item in scenarios
            if item["type"] == "commit_file"
        }
        for item in files:
            path = item["path"]
            if item["type"] != "commit_file" or not re.search(r"\.3dmap(?:\.zip)?$", path):
                continue
            name = path.rsplit("/", 1)[-1].removesuffix(".zip")
            stem = name.removesuffix(".3dmap")
            result.append(
                _entry(
                    "monash",
                    family,
                    name,
                    item["links"]["self"]["href"],
                    "voxel3d",
                    revision=revision,
                    download_bytes_claim=item.get("size"),
                    scenario_url=scenario_urls.get(stem),
                    directed=False,
                    lineage=f"movingai:warframe:{stem}"
                    if family == "warframe"
                    else f"monash:{family}:{stem}",
                    cost_model="euclidean_grid",
                    version="legacy_2018" if family == "warframe" else "socs2023",
                    catalog_url="https://benchmarks.pathfinding.ai/3d-benchmarks/voxel-maps/",
                )
            )
    if len([row for row in result if row["family"] != "warframe"]) != 46:
        raise ValueError("Monash inventory changed: expected 46 new voxel maps")
    return result


def ompl_catalog() -> list[dict[str, Any]]:
    """Catalog geometric OMPL.app scenes and independently reproducible demos."""
    repo = "ompl/omplapp"
    revision, tree = github_tree(repo)
    result = []
    for item in tree:
        path = item["path"]
        if path.startswith("resources/") and path.endswith(".cfg"):
            name = path.rsplit("/", 1)[-1]
            # Source cfg controls state-space semantics; a 2D environment may
            # still describe an SE(2) robot or a kinodynamic car.
            result.append(
                _entry(
                    "ompl",
                    "omplapp_configs",
                    name,
                    f"https://raw.githubusercontent.com/{repo}/{revision}/{path}",
                    "configuration_space",
                    revision=revision,
                    format="ompl_cfg",
                    lineage=f"omplapp:{path}",
                    catalog_url="https://ompl.kavrakilab.org/demos.html",
                )
            )
    revision, _ = github_tree("ompl/ompl")
    for name, source_file, dimension in (
        ("circle_grid", "CForestCircleGridBenchmark.cpp", 2),
        ("hypercube", "HypercubeBenchmark.cpp", None),
        ("kinematic_chain", "KinematicChainBenchmark.cpp", None),
        ("ppm_point", "Point2DPlanning.cpp", 2),
    ):
        result.append(
            _entry(
                "ompl",
                "geometric_demos",
                name,
                f"https://raw.githubusercontent.com/ompl/ompl/{revision}/demos/{source_file}",
                "continuous2d" if dimension == 2 else "configuration_space",
                revision=revision,
                format="generator_source",
                dimension=dimension,
                lineage=f"ompl:{name}",
                catalog_url="https://ompl.kavrakilab.org/demos.html",
            )
        )
    return result


def discover() -> dict[str, Any]:
    """Catalog five requested providers; every failed discovery is retained."""
    jobs: list[tuple[str, Callable[[], list[dict[str, Any]]]]] = [
        (f"movingai:{family}", lambda family=family: movingai_catalog(family))
        for family in FAMILIES
    ]
    jobs.extend(
        (
            ("dimacs", dimacs_catalog),
            ("barn", barn_catalog),
            ("monash", monash_catalog),
            ("ompl", ompl_catalog),
        )
    )
    entries: list[dict[str, Any]] = []
    failures = []
    with ThreadPoolExecutor(max_workers=4) as executor:
        futures = [(label, executor.submit(job)) for label, job in jobs]
        for label, future in futures:
            try:
                entries.extend(future.result())
            except Exception as exc:
                failures.append({"collection": label, "error": f"{type(exc).__name__}: {exc}"})
    return {
        "schema_version": SCHEMA,
        "created_at": datetime.now(timezone.utc).isoformat(),
        "entries": sorted(entries, key=lambda row: (row["source"], row["family"], row["name"])),
        "discovery_failures": failures,
        "scope": {
            "movingai": "all_current_2d_collections_and_corrected_3d",
            "dimacs": "core_road_distance_time_and_rome99",
            "barn": "300_static_worlds",
            "monash": "all_four_voxel_collections",
            "ompl": "geometric_demos_and_omplapp_cfg_inventory",
            "excluded": [
                "MAPF",
                "dynamic_navigation",
                "legacy_2d_duplicates",
                "restricted_DIMACS_contributions",
                "kinodynamic_execution",
            ],
        },
    }
