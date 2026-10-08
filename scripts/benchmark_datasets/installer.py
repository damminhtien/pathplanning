"""Install public benchmark inputs into an ignored, hash-indexed data root."""

from __future__ import annotations

from collections import Counter, defaultdict
from concurrent.futures import ThreadPoolExecutor, as_completed
import configparser
from dataclasses import dataclass, field
from datetime import datetime, timezone
from email.utils import parsedate_to_datetime
import gzip
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import re
import shutil
import stat
import tempfile
import threading
import time
from typing import Any, Callable
from urllib.error import HTTPError
from urllib.parse import unquote, urlsplit, urlunsplit
from urllib.request import Request, urlopen
import zipfile

from scripts.benchmark_datasets.catalog import SCHEMA as CATALOG_SCHEMA
from scripts.benchmark_datasets.catalog import discover, fetch
from scripts.benchmark_datasets.profiles import content_hash, profile_voxel_scenarios
from scripts.shortest_path_benchmark.workloads import MovingAIGrid, parse_scenario

INSTALL_SCHEMA = "pathplanning_dataset_installation_v1"
SOURCES = ("dimacs", "barn", "movingai", "monash", "ompl")
ENTRY_COUNTS = {"dimacs": 25, "barn": 300, "movingai": 853, "monash": 90, "ompl": 29}
DEFAULT_MAX_ASSET_BYTES = 8 << 30
DEFAULT_MAX_EXPANDED_BYTES = 16 << 30
DEFAULT_MIN_FREE_BYTES = 2 << 30
DEFAULT_WORKERS = 4
RETRIES = 3
CHUNK_SIZE = 1 << 20
_print_lock = threading.Lock()


@dataclass
class AssetSpec:
    """One output file, possibly shared by multiple dataset records."""

    asset_id: str
    source: str
    url: str
    destination_dir: str
    expected_name: str | None
    suffix: str | None
    role: str
    revision: str | None
    owners: set[str] = field(default_factory=set)
    source_paths: set[str] = field(default_factory=set)
    download_url: str | None = None


class InstallError(RuntimeError):
    """An input cannot be installed safely or completely."""


def _url_leaf(url: str) -> str:
    return unquote(urlsplit(url).path.rsplit("/", 1)[-1])


def _asset_id(source: str, url: str) -> str:
    return hashlib.sha256(f"{source}|{url}".encode()).hexdigest()


def _safe_relative(value: str) -> str:
    path = PurePosixPath(value)
    if (
        not value
        or path.is_absolute()
        or not path.parts
        or any(part in {"", ".", ".."} for part in path.parts)
        or "\\" in value
        or ":" in path.parts[0]
    ):
        raise InstallError(f"unsafe relative path: {value!r}")
    return path.as_posix()


def _resource_path(url: str, marker: str) -> str:
    parts = PurePosixPath(urlsplit(url).path).parts
    try:
        marker_index = parts.index(marker)
    except ValueError as exc:
        raise InstallError(f"source URL has no {marker}/ path: {url}") from exc
    return _safe_relative("/".join(parts[marker_index + 1 :]))


def _bitbucket_raw_file_url(url: str) -> str | None:
    """Convert a pinned Bitbucket API file URL to its raw web route."""
    parsed = urlsplit(url)
    if parsed.hostname != "api.bitbucket.org":
        return None
    parts = PurePosixPath(parsed.path).parts
    try:
        marker = parts.index("repositories")
    except ValueError:
        return None
    if len(parts) <= marker + 5 or parts[marker + 3] != "src":
        return None
    workspace, repository, _, revision = parts[marker + 1 : marker + 5]
    path = "/".join(parts[marker + 5 :])
    raw_path = f"/{workspace}/{repository}/raw/{revision}/{path}"
    return urlunsplit(("https", "bitbucket.org", raw_path, parsed.query, ""))


def _add_spec(
    specs: dict[str, AssetSpec],
    *,
    source: str,
    url: str,
    destination_dir: str,
    expected_name: str | None,
    suffix: str | None,
    role: str,
    revision: str | None,
    owner: str | None = None,
    source_path: str | None = None,
    download_url: str | None = None,
) -> str:
    asset_id = _asset_id(source, url)
    spec = specs.get(asset_id)
    safe_dir = _safe_relative(destination_dir)
    if spec is None:
        spec = AssetSpec(
            asset_id=asset_id,
            source=source,
            url=url,
            destination_dir=safe_dir,
            expected_name=expected_name,
            suffix=suffix,
            role=role,
            revision=revision,
            download_url=download_url,
        )
        specs[asset_id] = spec
    elif (spec.destination_dir, spec.expected_name, spec.download_url) != (
        safe_dir,
        expected_name,
        download_url,
    ):
        raise InstallError(f"one source URL maps to conflicting paths: {url}")
    if owner:
        spec.owners.add(owner)
    if source_path:
        spec.source_paths.add(source_path)
    return asset_id


def _dimacs_coordinate_url(url: str) -> str:
    if not url.endswith(".gr.gz"):
        raise InstallError(f"unexpected DIMACS graph URL: {url}")
    return url[: -len(".gr.gz")] + ".co.gz"


def _entry_layout(entry: dict[str, Any]) -> tuple[str, str, str | None]:
    """Return destination directory, source filename and expected archive suffix."""
    source, family = entry["source"], entry["family"]
    metadata = entry["metadata"]
    name = entry["name"]
    if source == "movingai":
        if entry["representation"] == "terrain2d":
            folder = "movingai-terrain"
        elif entry["representation"] == "voxel3d":
            folder = "movingai-3d"
        else:
            folder = f"movingai-v2/{family}"
        suffix = ".3dmap" if entry["representation"] == "voxel3d" else ".map"
        return folder, name, suffix
    if source == "dimacs":
        category = "distance" if family == "distance" else "travel_time"
        if family == "source_weight":
            category = "source_weight"
        return f"dimacs/{category}", name.removesuffix(".gz"), None
    if source == "barn":
        if metadata.get("path_url"):
            return "barn/worlds/BARN", name, ".world"
        return "barn", name, None
    if source == "monash":
        return f"monash/{family}", name, ".3dmap"
    if source == "ompl":
        if metadata.get("format") == "ompl_cfg":
            relative = _resource_path(entry["url"], "resources")
            return "ompl/omplapp/resources", relative, None
        relative = _resource_path(entry["url"], "demos")
        return "ompl/ompl/demos", relative, None
    raise InstallError(f"unsupported dataset source: {source}")


def _pinned_tree(repo: str, revision: str) -> list[dict[str, Any]]:
    payload = json.loads(
        fetch(
            f"https://api.github.com/repos/{repo}/git/trees/{revision}?recursive=1",
            maximum_bytes=8 << 20,
        )
    )
    if payload.get("truncated"):
        raise InstallError(f"source tree is truncated: {repo}@{revision}")
    return payload["tree"]


def build_asset_specs(
    catalog: dict[str, Any], sources: set[str], *, enforce_population_counts: bool = True
) -> tuple[list[AssetSpec], dict[str, set[str]]]:
    """Expand dataset entries into maps, queries, coordinates and source resources."""
    if catalog.get("schema_version") != CATALOG_SCHEMA:
        raise InstallError("unsupported dataset catalog schema")
    unknown = sources - set(SOURCES)
    if unknown:
        raise InstallError(f"unknown source(s): {', '.join(sorted(unknown))}")
    specs: dict[str, AssetSpec] = {}
    missing_by_dataset: dict[str, set[str]] = defaultdict(set)
    entries = [entry for entry in catalog["entries"] if entry["source"] in sources]
    known = Counter(entry["source"] for entry in entries)
    if enforce_population_counts and not catalog.get("discovery_failures"):
        for source in sources:
            if known[source] != ENTRY_COUNTS[source]:
                raise InstallError(
                    f"catalog for {source} contains {known[source]} entries; "
                    f"expected {ENTRY_COUNTS[source]}"
                )

    for entry in entries:
        source, identifier = entry["source"], entry["dataset_id"]
        metadata = entry["metadata"]
        directory, name, suffix = _entry_layout(entry)
        # OMPL.app configs are registered as part of the pinned 99-file
        # resource tree below so their paths match the referenced meshes.
        if not (source == "ompl" and metadata.get("format") == "ompl_cfg"):
            _add_spec(
                specs,
                source=source,
                url=entry["url"],
                destination_dir=directory,
                expected_name=name,
                suffix=suffix,
                role="dataset",
                revision=metadata.get("revision"),
                owner=identifier,
                source_path=entry["name"],
                download_url=_bitbucket_raw_file_url(entry["url"]) if source == "monash" else None,
            )

        scenario_url = metadata.get("scenario_url")
        if source in {"movingai", "monash"}:
            if scenario_url:
                scenario_suffix = ".3dscen" if entry["representation"] == "voxel3d" else ".scen"
                scenario_name = _url_leaf(scenario_url)
                if scenario_name.endswith(".zip"):
                    scenario_name = scenario_name[:-4]
                    if scenario_name.endswith("-scen"):
                        scenario_name = scenario_name[:-5] + ".scen"
                _add_spec(
                    specs,
                    source=source,
                    url=scenario_url,
                    destination_dir=directory,
                    expected_name=scenario_name,
                    suffix=scenario_suffix,
                    role="scenario",
                    revision=metadata.get("revision"),
                    owner=identifier,
                    download_url=(
                        _bitbucket_raw_file_url(scenario_url) if source == "monash" else None
                    ),
                )
            else:
                missing_by_dataset[identifier].add("scenario_not_linked_by_source")

        if source == "barn" and metadata.get("path_url"):
            path_name = _url_leaf(metadata["path_url"])
            _add_spec(
                specs,
                source=source,
                url=metadata["path_url"],
                destination_dir="barn/path_files",
                expected_name=path_name,
                suffix=None,
                role="supplied_path",
                revision=metadata.get("revision"),
                owner=identifier,
                source_path=f"path_files/{path_name}",
            )

        if source == "dimacs" and entry["family"] == "distance":
            coordinate_url = _dimacs_coordinate_url(entry["url"])
            coordinate_name = _url_leaf(coordinate_url).removesuffix(".gz")
            _add_spec(
                specs,
                source=source,
                url=coordinate_url,
                destination_dir="dimacs/coordinates",
                expected_name=coordinate_name,
                suffix=None,
                role="coordinates",
                revision=None,
                owner=identifier,
                source_path=coordinate_name,
            )

    if "ompl" in sources:
        ompl_entries = [entry for entry in entries if entry["source"] == "ompl"]
        cfg_owners: dict[str, str] = {}
        for entry in ompl_entries:
            if entry["metadata"].get("format") == "ompl_cfg":
                relative = _resource_path(entry["url"], "resources")
                cfg_owners[relative] = entry["dataset_id"]
        app_revision = next(
            entry["metadata"]["revision"]
            for entry in ompl_entries
            if entry["metadata"].get("format") == "ompl_cfg"
        )
        app_tree = _pinned_tree("ompl/omplapp", app_revision)
        resource_files = [
            item["path"]
            for item in app_tree
            if item["type"] == "blob" and item["path"].startswith("resources/")
        ]
        if len(resource_files) != 99:
            raise InstallError(
                f"OMPL.app resources changed: expected 99, found {len(resource_files)}"
            )
        for path in resource_files:
            relative = _safe_relative(path.removeprefix("resources/"))
            url = f"https://raw.githubusercontent.com/ompl/omplapp/{app_revision}/{path}"
            folder = PurePosixPath(relative).parent
            owners = {
                owner
                for cfg_path, owner in cfg_owners.items()
                if PurePosixPath(cfg_path).parent == folder
            }
            owners.update(owner for cfg_path, owner in cfg_owners.items() if cfg_path == path)
            _add_spec(
                specs,
                source="ompl",
                url=url,
                destination_dir=f"ompl/omplapp/resources/{folder.as_posix()}"
                if str(folder) != "."
                else "ompl/omplapp/resources",
                expected_name=PurePosixPath(relative).name,
                suffix=None,
                role="ompl_resource",
                revision=app_revision,
                source_path=path,
            )
            specs[_asset_id("ompl", url)].owners.update(owners)

        core_revision = next(
            entry["metadata"]["revision"]
            for entry in ompl_entries
            if entry["metadata"].get("format") == "generator_source"
        )
        core_tree = _pinned_tree("ompl/ompl", core_revision)
        required_core_paths = {"demos/KinematicChain.h", "tests/resources/ppm/floor.ppm"}
        available_core_paths = {item["path"] for item in core_tree if item["type"] == "blob"}
        if missing := required_core_paths - available_core_paths:
            raise InstallError(f"OMPL core resources are missing: {sorted(missing)}")
        for path in sorted(required_core_paths):
            owner = next(
                entry["dataset_id"]
                for entry in ompl_entries
                if entry["name"]
                == ("kinematic_chain" if path.endswith("KinematicChain.h") else "ppm_point")
            )
            url = f"https://raw.githubusercontent.com/ompl/ompl/{core_revision}/{path}"
            folder = (
                "ompl/ompl/demos" if path.startswith("demos/") else "ompl/ompl/tests/resources/ppm"
            )
            _add_spec(
                specs,
                source="ompl",
                url=url,
                destination_dir=folder,
                expected_name=PurePosixPath(path).name,
                suffix=None,
                role="ompl_demo_dependency",
                revision=core_revision,
                owner=owner,
                source_path=path,
            )

    result = sorted(specs.values(), key=lambda spec: (spec.source, spec.url, spec.role))
    # Identical regional coordinate files are installed once and associated with
    # the distance/time graphs that share them.
    entries_by_id = {entry["dataset_id"]: entry for entry in entries}
    for spec in result:
        if spec.source == "dimacs" and spec.role == "coordinates":
            region = next(entries_by_id[owner]["metadata"]["region"] for owner in spec.owners)
            spec.owners.update(
                entry["dataset_id"]
                for entry in entries
                if entry["source"] == "dimacs"
                and entry["metadata"].get("region") == region
                and entry["family"] in {"distance", "travel_time"}
            )
    return result, missing_by_dataset


def _cache_path(url: str, cache: Path) -> Path:
    leaf = re.sub(r"[^A-Za-z0-9._-]+", "_", _url_leaf(url)).strip("._") or "asset"
    return cache / f"{hashlib.sha256(url.encode()).hexdigest()}-{leaf}"


def _check_free_space(path: Path, minimum_free_bytes: int) -> None:
    path.mkdir(parents=True, exist_ok=True)
    if shutil.disk_usage(path).free < minimum_free_bytes:
        raise InstallError("minimum free disk space would be violated")


def _retry_after_seconds(value: str | None) -> float:
    if not value:
        return 60.0
    try:
        delay = float(value)
    except ValueError:
        try:
            retry_at = parsedate_to_datetime(value)
        except (TypeError, ValueError, OverflowError):
            return 60.0
        if retry_at.tzinfo is None:
            retry_at = retry_at.replace(tzinfo=timezone.utc)
        delay = (retry_at - datetime.now(timezone.utc)).total_seconds()
    return min(300.0, max(1.0, delay))


class _BitbucketRateLimiter:
    """Space Bitbucket requests and pause the host after a rate-limit response."""

    def __init__(self, interval_seconds: float = 1.25) -> None:
        self.interval_seconds = interval_seconds
        self.lock = threading.Lock()
        self.next_request_at = 0.0
        self.blocked_until = 0.0

    def open(self, request: Request, *, timeout: float) -> Any:
        host = urlsplit(request.full_url).hostname
        if host != "api.bitbucket.org":
            return urlopen(request, timeout=timeout)
        with self.lock:
            now = time.monotonic()
            ready_at = max(self.next_request_at, self.blocked_until)
            if ready_at > now:
                time.sleep(ready_at - now)
            self.next_request_at = time.monotonic() + self.interval_seconds
        try:
            return urlopen(request, timeout=timeout)
        except HTTPError as exc:
            if exc.code == 429:
                blocked_until = time.monotonic() + _retry_after_seconds(
                    exc.headers.get("Retry-After")
                )
                with self.lock:
                    self.blocked_until = max(self.blocked_until, blocked_until)
            raise


def download_to_cache(
    url: str,
    cache: Path,
    *,
    maximum_bytes: int,
    minimum_free_bytes: int,
    opener: Callable[..., Any] | None = None,
    request_url: str | None = None,
) -> Path:
    """Stream one source asset to a retryable URL cache without buffering it."""
    destination = _cache_path(url, cache)
    if destination.exists():
        if destination.stat().st_size > maximum_bytes:
            raise InstallError(f"cached asset exceeds size limit: {url}")
        _check_free_space(cache, minimum_free_bytes)
        return destination
    cache.mkdir(parents=True, exist_ok=True)
    open_url = opener or urlopen
    last_error: Exception | None = None
    for attempt in range(RETRIES):
        temporary = destination.with_name(destination.name + ".part")
        total = 0
        try:
            request = Request(
                request_url or url,
                headers={"User-Agent": "pathplanning-dataset-installer/1"},
            )
            with open_url(request, timeout=30) as response:
                length = response.headers.get("Content-Length")
                if length and int(length) > maximum_bytes:
                    raise InstallError(f"download size limit exceeded: {url}")
                with temporary.open("wb") as stream:
                    while chunk := response.read(CHUNK_SIZE):
                        total += len(chunk)
                        if total > maximum_bytes:
                            raise InstallError(f"download size limit exceeded: {url}")
                        if shutil.disk_usage(cache).free - len(chunk) < minimum_free_bytes:
                            raise InstallError("minimum free disk space would be violated")
                        stream.write(chunk)
                    stream.flush()
                    os.fsync(stream.fileno())
            if length and total != int(length):
                raise InstallError(f"download length mismatch: {url}")
            temporary.replace(destination)
            return destination
        except Exception as exc:
            last_error = exc
            temporary.unlink(missing_ok=True)
            if isinstance(exc, InstallError) and ("limit" in str(exc) or "space" in str(exc)):
                raise
            if attempt + 1 < RETRIES:
                time.sleep(0.5 * (2**attempt))
    raise InstallError(f"download failed after {RETRIES} attempts: {url}: {last_error}")


def _safe_zip_members(
    archive: zipfile.ZipFile, maximum_expanded_bytes: int
) -> list[zipfile.ZipInfo]:
    selected = []
    expanded = 0
    for item in archive.infolist():
        path = PurePosixPath(item.filename)
        if (
            path.is_absolute()
            or ".." in path.parts
            or "\\" in item.filename
            or not path.parts
            or ":" in path.parts[0]
        ):
            raise InstallError(f"unsafe path in zip archive: {item.filename!r}")
        if item.is_dir():
            continue
        mode = item.external_attr >> 16
        if stat.S_ISLNK(mode):
            raise InstallError(f"symbolic link in zip archive: {item.filename!r}")
        expanded += item.file_size
        if expanded > maximum_expanded_bytes:
            raise InstallError("expanded archive size limit exceeded")
        selected.append(item)
    return selected


def _stream_to_temporary(
    source: Any,
    destination_dir: Path,
    *,
    maximum_expanded_bytes: int,
    minimum_free_bytes: int = 0,
) -> tuple[Path, str, int]:
    destination_dir.mkdir(parents=True, exist_ok=True)
    fd, name = tempfile.mkstemp(prefix=".dataset-install-", suffix=".part", dir=destination_dir)
    digest = hashlib.sha256()
    total = 0
    path = Path(name)
    try:
        with os.fdopen(fd, "wb") as output:
            while chunk := source.read(CHUNK_SIZE):
                total += len(chunk)
                if total > maximum_expanded_bytes:
                    raise InstallError("expanded asset size limit exceeded")
                if shutil.disk_usage(destination_dir).free - len(chunk) < minimum_free_bytes:
                    raise InstallError("minimum free disk space would be violated")
                output.write(chunk)
                digest.update(chunk)
            output.flush()
            os.fsync(output.fileno())
        return path, digest.hexdigest(), total
    except Exception:
        path.unlink(missing_ok=True)
        raise


def materialize_asset(
    spec: AssetSpec,
    cached_file: Path,
    root: Path,
    *,
    maximum_expanded_bytes: int,
    minimum_free_bytes: int = 0,
) -> dict[str, Any]:
    """Verify or unpack one cached asset, then atomically publish its file."""
    destination_dir = root / Path(spec.destination_dir)
    destination_dir.resolve().relative_to(root.resolve())
    is_zip = zipfile.is_zipfile(cached_file)
    expects_zip = urlsplit(spec.url).path.lower().endswith(".zip")
    if expects_zip and not is_zip:
        raise InstallError(f"invalid zip archive: {spec.url}")
    is_gzip = not is_zip and (cached_file.suffix == ".gz" or spec.url.endswith(".gz"))
    if is_zip:
        with zipfile.ZipFile(cached_file) as archive:
            members = _safe_zip_members(archive, maximum_expanded_bytes)
            candidates = [
                item
                for item in members
                if spec.suffix and item.filename.lower().endswith(spec.suffix)
            ]
            if len(candidates) != 1:
                raise InstallError(
                    f"expected one {spec.suffix} payload in archive; found {len(candidates)}"
                )
            item = candidates[0]
            filename = spec.expected_name or PurePosixPath(item.filename).name
            if PurePosixPath(filename).name != filename:
                raise InstallError("archive output name must be a single filename")
            with archive.open(item) as source:
                temporary, digest, size = _stream_to_temporary(
                    source,
                    destination_dir,
                    maximum_expanded_bytes=maximum_expanded_bytes,
                    minimum_free_bytes=minimum_free_bytes,
                )
    else:
        filename = spec.expected_name or _url_leaf(spec.url)
        if is_gzip:
            if filename.endswith(".gz"):
                filename = filename[:-3]
            with gzip.open(cached_file, "rb") as source:
                temporary, digest, size = _stream_to_temporary(
                    source,
                    destination_dir,
                    maximum_expanded_bytes=maximum_expanded_bytes,
                    minimum_free_bytes=minimum_free_bytes,
                )
        else:
            with cached_file.open("rb") as source:
                temporary, digest, size = _stream_to_temporary(
                    source,
                    destination_dir,
                    maximum_expanded_bytes=maximum_expanded_bytes,
                    minimum_free_bytes=minimum_free_bytes,
                )

    relative_path = _safe_relative(f"{spec.destination_dir}/{filename}")
    destination = root / Path(relative_path)
    try:
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.parent.resolve().relative_to(root.resolve())
        if destination.exists() and destination.stat().st_size == size:
            if content_hash(destination) == digest:
                temporary.unlink(missing_ok=True)
                status = "verified_existing"
            else:
                temporary.replace(destination)
                status = "replaced_changed_content"
        else:
            temporary.replace(destination)
            status = "installed"
    except Exception:
        temporary.unlink(missing_ok=True)
        raise
    return {
        "relative_path": relative_path,
        "sha256": digest,
        "size_bytes": size,
        "status": status,
        "source_member": item.filename if is_zip else None,
    }


def _write_json(path: Path, data: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".part")
    temporary.write_text(json.dumps(data, indent=2, sort_keys=True, allow_nan=False) + "\n")
    temporary.replace(path)


def _attribution() -> dict[str, dict[str, Any]]:
    return {
        "dimacs": {
            "source": "https://www.diag.uniroma1.it/challenge9/download.shtml",
            "terms": "Public domain unless otherwise stated on the source page.",
            "citation": "9th DIMACS Implementation Challenge: Shortest Paths",
        },
        "movingai": {
            "source": "https://www.movingai.com/benchmarks/grids.html",
            "license": "Open Data Commons Attribution License",
            "citation": "Sturtevant, Benchmarks for Grid-Based Pathfinding, 2012",
        },
        "barn": {
            "source": "https://www.cs.utexas.edu/~xiao/BARN/BARN.html",
            "license_status": "check source dataset terms",
            "citation": "Perille et al., Benchmarking Metric Ground Navigation, 2020",
        },
        "monash": {
            "source": "https://benchmarks.pathfinding.ai/3d-benchmarks/voxel-maps/",
            "license_status": "check source dataset terms",
            "citation": "Nobes et al., 3D voxel pathfinding benchmarks, 2023",
        },
        "ompl": {
            "source": "https://ompl.kavrakilab.org/demos.html",
            "license_status": "check source repository terms",
            "citation": "Open Motion Planning Library demo resources",
        },
    }


def _dataset_record(entry: dict[str, Any]) -> dict[str, Any]:
    support = "registered_data_only"
    if entry["source"] == "movingai" and entry["representation"] == "grid2d":
        support = "movingai_land_v1"
    elif entry["source"] == "ompl" and entry["metadata"].get("format") == "ompl_cfg":
        support = "omplapp_config_reference_only"
    return {
        "dataset_id": entry["dataset_id"],
        "source": entry["source"],
        "family": entry["family"],
        "name": entry["name"],
        "representation": entry["representation"],
        "lineage": entry["metadata"].get("lineage"),
        "version": entry["metadata"].get("version", entry["metadata"].get("revision")),
        "revision": entry["metadata"].get("revision"),
        "source_url": entry["url"],
        "benchmark_support": support,
        "asset_ids": [],
        "missing_source_assets": [],
        "status": "pending",
        "errors": [],
    }


class IndexStore:
    """Serialize concurrent asset completions into an atomic local index."""

    def __init__(self, root: Path, catalog: dict[str, Any], selected_sources: set[str]) -> None:
        self.root = root
        self.path = root / "index.json"
        self.lock = threading.Lock()
        selected_ids = {
            entry["dataset_id"]
            for entry in catalog["entries"]
            if entry["source"] in selected_sources
        }
        if self.path.exists():
            self.data = json.loads(self.path.read_text())
            if self.data.get("schema_version") != INSTALL_SCHEMA:
                raise InstallError("unsupported existing installation index")
            self.data["datasets"] = {
                key: row for key, row in self.data["datasets"].items() if key not in selected_ids
            }
            self.data["assets"] = {
                key: row
                for key, row in self.data["assets"].items()
                if not (set(row.get("dataset_ids", [])) & selected_ids)
            }
        else:
            self.data = {"schema_version": INSTALL_SCHEMA, "datasets": {}, "assets": {}}
        for entry in catalog["entries"]:
            if entry["source"] in selected_sources:
                self.data["datasets"][entry["dataset_id"]] = _dataset_record(entry)
        self.data["sources"] = _attribution()
        self.data["created_at"] = self.data.get("created_at") or time.strftime(
            "%Y-%m-%dT%H:%M:%SZ", time.gmtime()
        )
        self.data["updated_at"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
        self.save()

    def save(self) -> None:
        _write_json(self.path, self.data)

    def complete(self, spec: AssetSpec, result: dict[str, Any]) -> None:
        with self.lock:
            self.data["assets"][spec.asset_id] = {
                "asset_id": spec.asset_id,
                "source": spec.source,
                "dataset_ids": sorted(spec.owners),
                "role": spec.role,
                "url": spec.url,
                "download_url": spec.download_url or spec.url,
                "revision": spec.revision,
                "source_paths": sorted(spec.source_paths),
                **result,
            }
            for owner in spec.owners:
                record = self.data["datasets"].get(owner)
                if record and spec.asset_id not in record["asset_ids"]:
                    record["asset_ids"].append(spec.asset_id)
            self.save()

    def fail(self, spec: AssetSpec, error: str) -> None:
        with self.lock:
            self.data["assets"][spec.asset_id] = {
                "asset_id": spec.asset_id,
                "source": spec.source,
                "dataset_ids": sorted(spec.owners),
                "role": spec.role,
                "url": spec.url,
                "download_url": spec.download_url or spec.url,
                "revision": spec.revision,
                "source_paths": sorted(spec.source_paths),
                "relative_path": None,
                "sha256": None,
                "size_bytes": None,
                "status": "error",
                "error": error,
            }
            for owner in spec.owners:
                record = self.data["datasets"].get(owner)
                if record:
                    record["errors"].append({"role": spec.role, "error": error})
                    record["asset_ids"].append(spec.asset_id)
            self.save()


def _finish_dataset_records(
    store: IndexStore, missing_by_dataset: dict[str, set[str]], selected_sources: set[str]
) -> None:
    with store.lock:
        for record in store.data["datasets"].values():
            if record["source"] not in selected_sources:
                continue
            record["missing_source_assets"] = sorted(
                missing_by_dataset.get(record["dataset_id"], set())
            )
            statuses = [
                store.data["assets"].get(asset_id, {}).get("status")
                for asset_id in record["asset_ids"]
            ]
            if record["errors"] or "error" in statuses:
                record["status"] = "error"
            elif any(
                status not in {"installed", "verified_existing", "replaced_changed_content"}
                for status in statuses
            ):
                record["status"] = "pending"
            elif record["missing_source_assets"]:
                record["status"] = "missing_source_asset"
            else:
                record["status"] = "installed"
        store.save()


def install_catalog(
    catalog: dict[str, Any],
    root: Path,
    *,
    sources: set[str] | None = None,
    workers: int = DEFAULT_WORKERS,
    cache: Path | None = None,
    maximum_asset_bytes: int = DEFAULT_MAX_ASSET_BYTES,
    maximum_expanded_bytes: int = DEFAULT_MAX_EXPANDED_BYTES,
    minimum_free_bytes: int = DEFAULT_MIN_FREE_BYTES,
    enforce_population_counts: bool = True,
) -> dict[str, Any]:
    """Install selected sources, retaining every outcome in the local index."""
    selected_sources = set(sources or SOURCES)
    if workers < 1 or workers > 32:
        raise ValueError("workers must be between 1 and 32")
    if min(maximum_asset_bytes, maximum_expanded_bytes, minimum_free_bytes) < 1:
        raise ValueError("byte limits must be positive")
    if catalog.get("discovery_failures"):
        discovery_failures = catalog["discovery_failures"]
    else:
        discovery_failures = []
    root = root.expanduser().resolve()
    root.mkdir(parents=True, exist_ok=True)
    _check_free_space(root, minimum_free_bytes)
    cache = (cache or root.parent / "dataset-characterization" / "cache").expanduser().resolve()
    store = IndexStore(root, catalog, selected_sources)
    rate_limiter = _BitbucketRateLimiter()
    _write_json(root / "catalog.json", catalog)
    for entry in catalog["entries"]:
        if entry["source"] in selected_sources:
            store.data["datasets"][entry["dataset_id"]] = _dataset_record(entry)
    specs, missing_by_dataset = build_asset_specs(
        catalog, selected_sources, enforce_population_counts=enforce_population_counts
    )

    paths: dict[str, str] = {}
    for spec in specs:
        suffix = spec.suffix or PurePosixPath(spec.expected_name or _url_leaf(spec.url)).suffix
        expected_name = spec.expected_name or f"<archive:{suffix}>"
        relative = _safe_relative(f"{spec.destination_dir}/{expected_name}")
        previous = paths.get(relative)
        if previous and previous != spec.asset_id:
            raise InstallError(f"different source assets target the same file: {relative}")
        paths[relative] = spec.asset_id

    print_lock = threading.Lock()

    def install_one(number: int, spec: AssetSpec) -> None:
        with print_lock:
            print(f"[{number}/{len(specs)}] {spec.source}: {_url_leaf(spec.url)}", flush=True)
        try:
            cached = download_to_cache(
                spec.url,
                cache,
                maximum_bytes=maximum_asset_bytes,
                minimum_free_bytes=minimum_free_bytes,
                opener=rate_limiter.open,
                request_url=spec.download_url,
            )
            result = materialize_asset(
                spec,
                cached,
                root,
                maximum_expanded_bytes=maximum_expanded_bytes,
                minimum_free_bytes=minimum_free_bytes,
            )
            store.complete(spec, result)
        except Exception as exc:
            cached = _cache_path(spec.url, cache)
            # A bad cached archive/stream must be retried from the source on the
            # next install attempt rather than poisoning resumable installs.
            if cached.exists() and isinstance(
                exc, (OSError, EOFError, gzip.BadGzipFile, zipfile.BadZipFile, InstallError)
            ):
                message = str(exc).lower()
                if any(
                    token in message for token in ("zip", "crc", "gzip", "truncated", "invalid")
                ):
                    cached.unlink(missing_ok=True)
            store.fail(spec, f"{type(exc).__name__}: {exc}")

    if specs:
        with ThreadPoolExecutor(max_workers=workers) as executor:
            futures = [
                executor.submit(install_one, index, spec) for index, spec in enumerate(specs, 1)
            ]
            for future in as_completed(futures):
                future.result()

    _finish_dataset_records(store, missing_by_dataset, selected_sources)
    store.data["discovery_failures"] = discovery_failures
    store.data["updated_at"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
    store.save()
    return store.data


def status_report(root: Path, catalog: dict[str, Any] | None = None) -> dict[str, Any]:
    """Summarize catalog denominators, installed records and file outcomes."""
    root = root.expanduser().resolve()
    index_path = root / "index.json"
    if index_path.exists():
        index = json.loads(index_path.read_text())
        if index.get("schema_version") != INSTALL_SCHEMA:
            raise InstallError("unsupported installation index schema")
        active_catalog = (
            json.loads((root / "catalog.json").read_text())
            if (root / "catalog.json").exists()
            else catalog
        )
    else:
        index = {"datasets": {}, "assets": {}}
        active_catalog = catalog
    if active_catalog is None:
        active_catalog = discover()
    datasets = index.get("datasets", {})
    assets = index.get("assets", {})
    asset_status: dict[str, str] = {}
    for asset_id, asset in assets.items():
        status = asset.get("status", "unknown")
        if status in {"installed", "verified_existing", "replaced_changed_content"}:
            try:
                if not _path_for_asset(root, asset).is_file():
                    status = "missing_file"
            except (KeyError, OSError, ValueError, InstallError):
                status = "missing_file"
        asset_status[asset_id] = status
    by_source: dict[str, dict[str, int]] = {}
    for source in SOURCES:
        counters: Counter[str] = Counter()
        expected = [entry for entry in active_catalog["entries"] if entry["source"] == source]
        for entry in expected:
            row = datasets.get(entry["dataset_id"])
            if row is None:
                status = "not_installed"
            else:
                linked_statuses = [
                    asset_status.get(asset_id, "missing_file")
                    for asset_id in row.get("asset_ids", [])
                ]
                if "missing_file" in linked_statuses or (
                    row.get("status") in {"installed", "missing_source_asset"}
                    and not linked_statuses
                ):
                    status = "missing_file"
                else:
                    status = row.get("status", "pending")
            counters[status] += 1
        by_source[source] = {"catalog": len(expected), **dict(sorted(counters.items()))}
    asset_counts = Counter(asset_status.values())
    return {
        "catalog_entries": len(active_catalog["entries"]),
        "discovery_failures": active_catalog.get("discovery_failures", []),
        "by_source": by_source,
        "assets": dict(sorted(asset_counts.items())),
    }


def _path_for_asset(root: Path, record: dict[str, Any]) -> Path:
    relative = _safe_relative(record["relative_path"])
    target = root / Path(relative)
    target.resolve().relative_to(root.resolve())
    return target


def _verify_scenario_map_reference(
    scenario_path: Path,
    map_path: Path,
    dataset_root: Path,
    representation: str,
) -> None:
    """Ensure every map name in a linked scenario resolves to its paired map."""
    root = dataset_root.resolve()
    map_file = map_path.resolve()

    def check_reference(value: str) -> None:
        reference = PurePosixPath(value)
        if (
            not value
            or reference.is_absolute()
            or not reference.parts
            or ".." in reference.parts
            or "\\" in value
        ):
            raise ValueError(f"unsafe map reference in scenario: {value!r}")
        candidate = scenario_path.parent.joinpath(*reference.parts).resolve()
        candidate.relative_to(root)
        if candidate != map_file:
            raise ValueError(
                f"scenario map reference {value!r} does not resolve to {map_path.name}"
            )
        if not candidate.is_file():
            raise ValueError(f"scenario map does not exist: {candidate}")

    with scenario_path.open(encoding="ascii") as stream:
        version = stream.readline().strip()
        if representation == "voxel3d":
            if version not in {"version 1", "version 2"}:
                raise ValueError(f"unsupported voxel scenario header: {version!r}")
            check_reference(stream.readline().strip())
            return
        if version not in {"version 1", "version 1.0"}:
            raise ValueError(f"unsupported grid scenario header: {version!r}")
        for line_number, line in enumerate(stream, start=2):
            fields = line.split()
            if not fields:
                continue
            if len(fields) != 9:
                raise ValueError(f"scenario line {line_number} must have nine fields")
            check_reference(fields[1])


def _verify_ompl_references(
    root: Path, catalog: dict[str, Any], index: dict[str, Any]
) -> list[str]:
    errors = []
    asset_by_path = {
        source_path: asset
        for asset in index["assets"].values()
        if asset.get("source") == "ompl" and asset.get("relative_path")
        for source_path in asset.get("source_paths", [])
    }
    for entry in catalog["entries"]:
        if entry["source"] != "ompl" or entry["metadata"].get("format") != "ompl_cfg":
            continue
        source_path = _resource_path(entry["url"], "resources")
        cfg_asset = asset_by_path.get(f"resources/{source_path}")
        if cfg_asset is None:
            errors.append(f"OMPL config is missing: {source_path}")
            continue
        cfg_file = _path_for_asset(root, cfg_asset)
        parser = configparser.ConfigParser(strict=False)
        parser.read(cfg_file)
        problem = parser["problem"] if "problem" in parser else {}
        folder = PurePosixPath(source_path).parent
        for key in ("robot", "world"):
            value = problem.get(key)
            if value:
                dependency = PurePosixPath("resources") / folder / PurePosixPath(value)
                normalized = PurePosixPath(os.path.normpath(dependency.as_posix()))
                if normalized.is_absolute() or ".." in normalized.parts:
                    errors.append(f"unsafe OMPL {key} path in {source_path}: {value}")
                    continue
                dependency_asset = asset_by_path.get(normalized.as_posix())
                if dependency_asset is None:
                    errors.append(f"OMPL {key} asset missing for {source_path}: {value}")
                elif not _path_for_asset(root, dependency_asset).is_file():
                    errors.append(f"OMPL {key} asset is absent on disk for {source_path}: {value}")
    return errors


def verify_installation(root: Path, *, sources: set[str] | None = None) -> dict[str, Any]:
    """Hash every installed file and validate map, query and mesh references."""
    selected_sources = set(sources or SOURCES)
    root = root.expanduser().resolve()
    index_path, catalog_path = root / "index.json", root / "catalog.json"
    if not index_path.is_file() or not catalog_path.is_file():
        return {
            "valid": False,
            "checked_assets": 0,
            "errors": ["catalog or index is missing"],
            "warnings": [],
        }
    index, catalog = json.loads(index_path.read_text()), json.loads(catalog_path.read_text())
    errors: list[str] = []
    warnings: list[str] = []
    checked = 0
    for asset in index.get("assets", {}).values():
        if asset.get("source") not in selected_sources:
            continue
        if asset.get("status") not in {
            "installed",
            "verified_existing",
            "replaced_changed_content",
        }:
            if asset.get("status") == "not_found_from_source":
                warnings.append(
                    f"source did not link {asset['source']} {asset['role']}: {asset['url']}"
                )
            else:
                errors.append(f"asset is not installed ({asset['status']}): {asset['url']}")
            continue
        try:
            path = _path_for_asset(root, asset)
            if not path.is_file():
                errors.append(f"installed file is absent: {asset['relative_path']}")
                continue
            checked += 1
            if path.stat().st_size != asset["size_bytes"] or content_hash(path) != asset["sha256"]:
                errors.append(f"hash/size mismatch: {asset['relative_path']}")
        except (KeyError, OSError, ValueError, InstallError) as exc:
            errors.append(f"invalid asset record: {exc}")

    for entry in catalog["entries"]:
        if entry["source"] not in selected_sources:
            continue
        if entry["source"] != "movingai":
            continue
        row = index.get("datasets", {}).get(entry["dataset_id"], {})
        map_assets = [index["assets"].get(asset_id) for asset_id in row.get("asset_ids", [])]
        map_asset = next(
            (asset for asset in map_assets if asset and asset["role"] == "dataset"), None
        )
        scenario_asset = next(
            (asset for asset in map_assets if asset and asset["role"] == "scenario"), None
        )
        if map_asset is None:
            errors.append(f"MovingAI map is missing: {entry['name']}")
            continue
        if entry["metadata"].get("scenario_url") and scenario_asset is None:
            errors.append(f"MovingAI map/scenario pair is incomplete: {entry['name']}")
            continue
        try:
            map_path = _path_for_asset(root, map_asset)
            if scenario_asset is not None:
                scenario_path = _path_for_asset(root, scenario_asset)
                _verify_scenario_map_reference(
                    scenario_path,
                    map_path,
                    root,
                    entry["representation"],
                )
                if entry["representation"] == "grid2d":
                    grid = MovingAIGrid.from_file(map_path)
                    parse_scenario(scenario_path, root / "movingai-v2", family=entry["family"])
                    if grid.map_sha256 != map_asset["sha256"]:
                        errors.append(
                            f"MovingAI map parser hash differs from index: {entry['name']}"
                        )
        except Exception as exc:
            errors.append(
                f"MovingAI reference/parser rejected {entry['name']}: {type(exc).__name__}: {exc}"
            )

    if "ompl" in selected_sources:
        errors.extend(_verify_ompl_references(root, catalog, index))
    for entry in catalog["entries"]:
        if entry["source"] not in selected_sources:
            continue
        if entry["source"] != "monash":
            continue
        record = index.get("datasets", {}).get(entry["dataset_id"], {})
        map_assets = [index["assets"].get(asset_id) for asset_id in record.get("asset_ids", [])]
        voxel_asset = next(
            (asset for asset in map_assets if asset and asset["role"] == "dataset"), None
        )
        scenario_asset = next(
            (asset for asset in map_assets if asset and asset["role"] == "scenario"), None
        )
        if voxel_asset is None:
            errors.append(f"Monash voxel map is missing: {entry['name']}")
            continue
        if scenario_asset is None:
            continue
        try:
            with _path_for_asset(root, voxel_asset).open(encoding="ascii") as stream:
                header = stream.readline().split()
            if len(header) != 4 or header[0] not in {"voxel", "rev_voxel"}:
                raise ValueError("invalid voxel header")
            dimensions = [int(value) for value in header[1:]]
            scenario_path = _path_for_asset(root, scenario_asset)
            _verify_scenario_map_reference(
                scenario_path, _path_for_asset(root, voxel_asset), root, "voxel3d"
            )
            profile_voxel_scenarios(scenario_path, dimensions)
        except Exception as exc:
            errors.append(
                f"Monash scenario rejected for {entry['name']}: {type(exc).__name__}: {exc}"
            )

    expected_entries = Counter(
        entry["source"] for entry in catalog["entries"] if entry["source"] in selected_sources
    )
    observed_entries = Counter(row["source"] for row in index.get("datasets", {}).values())
    for source, expected in expected_entries.items():
        if observed_entries[source] < expected:
            errors.append(
                f"catalog registration incomplete for {source}: {observed_entries[source]}/{expected}"
            )
    for entry in catalog["entries"]:
        if entry["source"] not in selected_sources:
            continue
        record = index.get("datasets", {}).get(entry["dataset_id"])
        if record is None:
            errors.append(f"dataset is not registered: {entry['source']}/{entry['name']}")
        elif record.get("status") in {"pending", "error"}:
            errors.append(
                f"dataset is incomplete ({record['status']}): {entry['source']}/{entry['name']}"
            )
        elif record.get("status") == "missing_source_asset":
            missing = ", ".join(record.get("missing_source_assets", []))
            warnings.append(f"source-linked asset unavailable for {entry['name']}: {missing}")
    return {"valid": not errors, "checked_assets": checked, "errors": errors, "warnings": warnings}


def load_catalog(path: Path | None, root: Path) -> dict[str, Any]:
    """Use the frozen survey catalog when present; discover on a fresh checkout."""
    if path is not None:
        return json.loads(path.read_text())
    frozen = root.expanduser().resolve().parent / "dataset-characterization" / "catalog.json"
    if frozen.is_file():
        return json.loads(frozen.read_text())
    return discover()
