"""Installer tests use tiny local assets and never contact dataset hosts."""

from __future__ import annotations

import gzip
import hashlib
from io import BytesIO
from pathlib import Path
from urllib.error import HTTPError
from urllib.request import Request
import zipfile

import pytest

from scripts.benchmark_datasets.catalog import SCHEMA
from scripts.benchmark_datasets.installer import (
    AssetSpec,
    InstallError,
    _bitbucket_raw_file_url,
    _retry_after_seconds,
    _verify_ompl_references,
    _verify_scenario_map_reference,
    build_asset_specs,
    download_to_cache,
    install_catalog,
    materialize_asset,
    status_report,
    verify_installation,
)


class _Response(BytesIO):
    def __init__(self, data: bytes) -> None:
        super().__init__(data)
        self.headers = {"Content-Length": str(len(data))}

    def __enter__(self) -> _Response:
        return self

    def __exit__(self, *args: object) -> None:
        self.close()


def _entry(
    source: str,
    name: str,
    url: str,
    representation: str,
    *,
    family: str = "maze",
    **metadata: object,
) -> dict[str, object]:
    return {
        "dataset_id": hashlib.sha256(f"{source}|{family}|{name}|{url}".encode()).hexdigest(),
        "source": source,
        "family": family,
        "name": name,
        "url": url,
        "representation": representation,
        "metadata": metadata,
    }


def _catalog(*entries: dict[str, object]) -> dict[str, object]:
    return {"schema_version": SCHEMA, "entries": list(entries), "discovery_failures": []}


def _zip(name: str, content: bytes) -> bytes:
    buffer = BytesIO()
    with zipfile.ZipFile(buffer, "w") as archive:
        archive.writestr(name, content)
    return buffer.getvalue()


def test_full_grid_asset_install_resumes_and_verifies_parser_pair(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    import scripts.benchmark_datasets.installer as installer

    grid = b"type octile\nheight 2\nwidth 2\nmap\n..\n..\n"
    scenario = b"version 1\n0 tiny.map 2 2 0 0 1 1 1.41421356\n"
    payloads = {
        "https://example.test/tiny.map.zip": _zip("tiny.map", grid),
        "https://example.test/tiny.map-scen.zip": _zip("tiny.map.scen", scenario),
    }
    calls: list[str] = []

    def open_url(request: object, timeout: int) -> _Response:
        url = request.full_url  # type: ignore[attr-defined]
        calls.append(url)
        return _Response(payloads[url])

    monkeypatch.setattr(installer, "urlopen", open_url)
    entry = _entry(
        "movingai",
        "tiny.map",
        "https://example.test/tiny.map.zip",
        "grid2d",
        scenario_url="https://example.test/tiny.map-scen.zip",
        lineage="movingai:maze:tiny",
    )
    catalog = _catalog(entry)
    root, cache = tmp_path / "datasets", tmp_path / "cache"
    result = install_catalog(
        catalog,
        root,
        sources={"movingai"},
        cache=cache,
        minimum_free_bytes=1,
        enforce_population_counts=False,
    )
    assert result["datasets"][entry["dataset_id"]]["status"] == "installed"
    assert (root / "movingai-v2/maze/tiny.map").read_bytes() == grid
    assert (root / "movingai-v2/maze/tiny.map.scen").read_bytes() == scenario
    assert verify_installation(root)["valid"]
    install_catalog(
        catalog,
        root,
        sources={"movingai"},
        cache=cache,
        minimum_free_bytes=1,
        enforce_population_counts=False,
    )
    assert len(calls) == 2
    assert status_report(root, catalog)["by_source"]["movingai"]["installed"] == 1


def test_downloader_retries_transient_failure_and_reuses_complete_cache(
    tmp_path: Path,
) -> None:
    calls = 0

    def open_url(request: object, timeout: int) -> _Response:
        nonlocal calls
        calls += 1
        if calls == 1:
            raise TimeoutError("temporary connection timeout")
        return _Response(b"complete payload")

    cache = tmp_path / "cache"
    result = download_to_cache(
        "https://example.test/data.map",
        cache,
        maximum_bytes=100,
        minimum_free_bytes=1,
        opener=open_url,
    )
    assert result.read_bytes() == b"complete payload"
    assert list(cache.glob("*.part")) == []
    assert (
        download_to_cache(
            "https://example.test/data.map",
            cache,
            maximum_bytes=100,
            minimum_free_bytes=1,
            opener=lambda *_args, **_kwargs: pytest.fail("complete cache should be reused"),
        )
        == result
    )
    assert calls == 2


def test_bitbucket_rate_limit_honors_retry_after(monkeypatch: pytest.MonkeyPatch) -> None:
    import scripts.benchmark_datasets.installer as installer

    error = HTTPError(
        "https://api.bitbucket.org/example",
        429,
        "too many requests",
        {"Retry-After": "45"},
        None,
    )

    def reject(*args: object, **kwargs: object) -> None:
        raise error

    monkeypatch.setattr(installer, "urlopen", reject)
    limiter = installer._BitbucketRateLimiter(interval_seconds=0)
    with pytest.raises(HTTPError):
        limiter.open(Request("https://api.bitbucket.org/example"), timeout=1)
    assert limiter.blocked_until > 0
    assert _retry_after_seconds("45") == 45
    assert _retry_after_seconds(None) == 60


def test_download_size_limit_and_interrupted_file_cleanup(tmp_path: Path) -> None:
    def too_large(request: object, timeout: int) -> _Response:
        return _Response(b"too large")

    with pytest.raises(InstallError, match="size limit"):
        download_to_cache(
            "https://example.test/data.map",
            tmp_path / "cache",
            maximum_bytes=2,
            minimum_free_bytes=1,
            opener=too_large,
        )
    assert list((tmp_path / "cache").glob("*.part")) == []


def test_zip_path_traversal_is_rejected_without_publishing_payload(tmp_path: Path) -> None:
    archive = tmp_path / "bad.zip"
    archive.write_bytes(_zip("../escape.map", b"blocked"))
    spec = AssetSpec(
        "id",
        "movingai",
        "https://example.test/bad.zip",
        "movingai-v2/maze",
        "safe.map",
        ".map",
        "dataset",
        None,
    )
    with pytest.raises(InstallError, match="unsafe path"):
        materialize_asset(spec, archive, tmp_path / "out", maximum_expanded_bytes=1024)
    assert not (tmp_path / "escape.map").exists()
    assert not (tmp_path / "out/movingai-v2/maze/safe.map").exists()


def test_gzip_graph_is_streamed_and_expansion_limit_is_enforced(tmp_path: Path) -> None:
    source = tmp_path / "tiny.gr.gz"
    source.write_bytes(gzip.compress(b"p sp 1 0\n"))
    spec = AssetSpec(
        "id", "dimacs", source.as_uri(), "dimacs/distance", "tiny.gr", None, "dataset", None
    )
    with pytest.raises(InstallError, match="expanded asset size limit"):
        materialize_asset(spec, source, tmp_path / "out", maximum_expanded_bytes=2)
    installed = materialize_asset(spec, source, tmp_path / "out", maximum_expanded_bytes=1024)
    assert installed["relative_path"] == "dimacs/distance/tiny.gr"
    assert (tmp_path / "out/dimacs/distance/tiny.gr").read_bytes() == b"p sp 1 0\n"


def test_coordinate_specs_are_shared_across_distance_and_time_graphs() -> None:
    distance = _entry(
        "dimacs",
        "USA-road-d.NY.gr.gz",
        "https://example.test/data/USA-road-d/USA-road-d.NY.gr.gz",
        "directed_graph",
        family="distance",
        region="NY",
        lineage="dimacs:tiger:NY",
    )
    travel = _entry(
        "dimacs",
        "USA-road-t.NY.gr.gz",
        "https://example.test/data/USA-road-t/USA-road-t.NY.gr.gz",
        "directed_graph",
        family="travel_time",
        region="NY",
        lineage="dimacs:tiger:NY",
    )
    specs, _ = build_asset_specs(
        _catalog(distance, travel), {"dimacs"}, enforce_population_counts=False
    )
    coordinate = next(spec for spec in specs if spec.role == "coordinates")
    assert coordinate.destination_dir == "dimacs/coordinates"
    assert coordinate.owners == {distance["dataset_id"], travel["dataset_id"]}


def test_monash_download_uses_pinned_raw_url_but_keeps_source_cache_key() -> None:
    revision = "fe6351b0700a0f4e75d0bd79ce3bf5478bc60c94"
    source_url = (
        "https://api.bitbucket.org/2.0/repositories/shortestpathlab/benchmarks/src/"
        f"{revision}/voxel-maps/descent/map_files/level01.3dmap.zip"
    )
    raw_url = (
        "https://bitbucket.org/shortestpathlab/benchmarks/raw/"
        f"{revision}/voxel-maps/descent/map_files/level01.3dmap.zip"
    )
    assert _bitbucket_raw_file_url(source_url) == raw_url
    entry = _entry(
        "monash",
        "level01.3dmap",
        source_url,
        "voxel3d",
        family="descent",
        revision=revision,
    )
    specs, _ = build_asset_specs(_catalog(entry), {"monash"}, enforce_population_counts=False)
    dataset_spec = next(spec for spec in specs if spec.role == "dataset")
    assert dataset_spec.url == source_url
    assert dataset_spec.download_url == raw_url


def test_monash_missing_source_scenario_is_registered_as_a_gap(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    import scripts.benchmark_datasets.installer as installer

    payload = b"voxel 2 2 2\n0 0 0\n"
    monkeypatch.setattr(installer, "urlopen", lambda *_args, **_kwargs: _Response(payload))
    entry = _entry(
        "monash",
        "level27.3dmap",
        "https://example.test/level27.3dmap",
        "voxel3d",
        family="descent",
        lineage="monash:descent:level27",
        scenario_url=None,
    )
    catalog = _catalog(entry)
    result = install_catalog(
        catalog,
        tmp_path / "datasets",
        sources={"monash"},
        cache=tmp_path / "cache",
        minimum_free_bytes=1,
        enforce_population_counts=False,
    )
    record = result["datasets"][entry["dataset_id"]]
    assert record["status"] == "missing_source_asset"
    assert record["missing_source_assets"] == ["scenario_not_linked_by_source"]
    assert verify_installation(tmp_path / "datasets")["valid"]
    assert (
        status_report(tmp_path / "datasets", catalog)["by_source"]["monash"]["missing_source_asset"]
        == 1
    )


def test_ompl_verification_reports_missing_mesh_reference(tmp_path: Path) -> None:
    cfg = tmp_path / "ompl/omplapp/resources/2D/test.cfg"
    mesh = tmp_path / "ompl/omplapp/resources/2D/robot.dae"
    cfg.parent.mkdir(parents=True)
    cfg.write_text("[problem]\nrobot = robot.dae\nworld = missing.dae\n")
    mesh.write_bytes(b"mesh")
    entry = _entry(
        "ompl",
        "test.cfg",
        "https://raw.githubusercontent.com/ompl/omplapp/tree/resources/2D/test.cfg",
        "configuration_space",
        family="omplapp_configs",
        format="ompl_cfg",
    )
    cfg_record = {
        "source": "ompl",
        "source_paths": ["resources/2D/test.cfg"],
        "relative_path": "ompl/omplapp/resources/2D/test.cfg",
    }
    mesh_record = {
        "source": "ompl",
        "source_paths": ["resources/2D/robot.dae"],
        "relative_path": "ompl/omplapp/resources/2D/robot.dae",
    }
    errors = _verify_ompl_references(
        tmp_path,
        _catalog(entry),
        {"assets": {"cfg": cfg_record, "mesh": mesh_record}},
    )
    assert errors == ["OMPL world asset missing for 2D/test.cfg: missing.dae"]


def test_voxel_scenario_must_reference_its_paired_map(tmp_path: Path) -> None:
    map_path = tmp_path / "movingai-3d/tiny.3dmap"
    scenario_path = tmp_path / "movingai-3d/tiny.3dscen"
    map_path.parent.mkdir(parents=True)
    map_path.write_text("voxel 2 2 2\n0 0 0\n")
    scenario_path.write_text("version 1\ntiny.3dmap\n0 0 0 1 1 1 1.732 1\n")
    _verify_scenario_map_reference(scenario_path, map_path, tmp_path, "voxel3d")
    scenario_path.write_text("version 1\nother.3dmap\n")
    with pytest.raises(ValueError, match="does not resolve"):
        _verify_scenario_map_reference(scenario_path, map_path, tmp_path, "voxel3d")


def test_corrupt_zip_cache_is_discarded_and_retried_from_source(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    import scripts.benchmark_datasets.installer as installer

    voxel = b"voxel 2 2 2\n0 0 0\n"
    payload = _zip("level27.3dmap", voxel)
    calls = 0

    def open_url(request: object, timeout: int) -> _Response:
        nonlocal calls
        calls += 1
        return _Response(payload)

    monkeypatch.setattr(installer, "urlopen", open_url)
    entry = _entry(
        "monash",
        "level27.3dmap",
        "https://example.test/level27.3dmap.zip",
        "voxel3d",
        family="descent",
        scenario_url=None,
    )
    cache = tmp_path / "cache"
    cached = installer._cache_path(entry["url"], cache)
    cache.mkdir()
    cached.write_bytes(b"corrupt cached zip")
    result = install_catalog(
        _catalog(entry),
        tmp_path / "datasets",
        sources={"monash"},
        cache=cache,
        minimum_free_bytes=1,
        enforce_population_counts=False,
    )
    assert result["datasets"][entry["dataset_id"]]["status"] == "error"
    assert not cached.exists()
    result = install_catalog(
        _catalog(entry),
        tmp_path / "datasets",
        sources={"monash"},
        cache=cache,
        minimum_free_bytes=1,
        enforce_population_counts=False,
    )
    assert result["datasets"][entry["dataset_id"]]["status"] == "missing_source_asset"
    assert calls == 1
    assert (tmp_path / "datasets/monash/descent/level27.3dmap").read_bytes() == voxel


def test_conflicting_dataset_target_paths_are_rejected_before_download(tmp_path: Path) -> None:
    first = _entry(
        "movingai", "duplicate.map", "https://example.test/first.zip", "grid2d", family="maze"
    )
    second = _entry(
        "movingai", "duplicate.map", "https://example.test/second.zip", "grid2d", family="maze"
    )
    with pytest.raises(InstallError, match="target the same file"):
        install_catalog(
            _catalog(first, second),
            tmp_path / "datasets",
            sources={"movingai"},
            minimum_free_bytes=1,
            enforce_population_counts=False,
        )


def test_partial_source_install_can_be_verified_independently(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    import scripts.benchmark_datasets.installer as installer

    grid = b"type octile\nheight 1\nwidth 1\nmap\n.\n"
    scenario = b"version 1\n0 tiny.map 1 1 0 0 0 0 0\n"
    payloads = {
        "https://example.test/tiny.map.zip": _zip("tiny.map", grid),
        "https://example.test/tiny.map-scen.zip": _zip("tiny.map.scen", scenario),
    }
    monkeypatch.setattr(
        installer,
        "urlopen",
        lambda request, timeout: _Response(payloads[request.full_url]),  # type: ignore[attr-defined]
    )
    movingai = _entry(
        "movingai",
        "tiny.map",
        "https://example.test/tiny.map.zip",
        "grid2d",
        scenario_url="https://example.test/tiny.map-scen.zip",
    )
    dimacs = _entry(
        "dimacs", "graph.gr", "https://example.test/graph.gr", "directed_graph", family="distance"
    )
    catalog = _catalog(movingai, dimacs)
    root = tmp_path / "datasets"
    install_catalog(
        catalog,
        root,
        sources={"movingai"},
        cache=tmp_path / "cache",
        minimum_free_bytes=1,
        enforce_population_counts=False,
    )
    assert verify_installation(root, sources={"movingai"})["valid"]
    assert not verify_installation(root)["valid"]


def test_status_reports_a_missing_installed_companion_file(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    import scripts.benchmark_datasets.installer as installer

    grid = b"type octile\nheight 1\nwidth 1\nmap\n.\n"
    scenario = b"version 1\n0 tiny.map 1 1 0 0 0 0 0\n"
    payloads = {
        "https://example.test/tiny.map.zip": _zip("tiny.map", grid),
        "https://example.test/tiny.map-scen.zip": _zip("tiny.map.scen", scenario),
    }
    monkeypatch.setattr(
        installer,
        "urlopen",
        lambda request, timeout: _Response(payloads[request.full_url]),  # type: ignore[attr-defined]
    )
    entry = _entry(
        "movingai",
        "tiny.map",
        "https://example.test/tiny.map.zip",
        "grid2d",
        scenario_url="https://example.test/tiny.map-scen.zip",
    )
    catalog = _catalog(entry)
    root = tmp_path / "datasets"
    install_catalog(
        catalog,
        root,
        sources={"movingai"},
        cache=tmp_path / "cache",
        minimum_free_bytes=1,
        enforce_population_counts=False,
    )
    (root / "movingai-v2/maze/tiny.map.scen").unlink()
    report = status_report(root, catalog)
    assert report["by_source"]["movingai"]["missing_file"] == 1
    assert report["assets"]["missing_file"] == 1
    (root / "movingai-v2/maze/tiny.map").write_bytes(b"modified map")
    verification = verify_installation(root)
    assert any("hash/size mismatch" in error for error in verification["errors"])
