"""Installer tests use tiny local assets and never contact dataset hosts."""

from __future__ import annotations

import hashlib
from io import BytesIO
from pathlib import Path
import zipfile

import pytest

from scripts.benchmark_datasets.catalog import SCHEMA
from scripts.benchmark_datasets.installer import (
    AssetSpec,
    InstallError,
    materialize_asset,
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
