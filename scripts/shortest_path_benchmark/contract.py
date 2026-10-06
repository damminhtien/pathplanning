"""Version 2 records and durable files for shortest-path experiments.

This module leaves ``scripts.benchmark_contract`` v1 unchanged. Integer work
and memory counters remain Python integers all the way to JSON serialization.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
from pathlib import Path
import tempfile
from typing import Any, Mapping

from scripts.benchmark_contract import environment_metadata, source_provenance, utc_now

SCHEMA_VERSION_V2 = "pathplanning_shortest_path_v2"
UINT64_MAX = (1 << 64) - 1
NAMESPACES = ("input", "outcome", "work", "memory", "timing", "capabilities", "provenance")


def _check_json(value: Any, path: str = "record") -> None:
    if value is None or isinstance(value, (str, bool)):
        return
    if isinstance(value, int):
        if abs(value) > UINT64_MAX:
            raise ValueError(f"{path} exceeds uint64 range")
        return
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ValueError(f"{path} must be finite or explicitly null with a reason")
        return
    if isinstance(value, list):
        for index, item in enumerate(value):
            _check_json(item, f"{path}[{index}]")
        return
    if isinstance(value, dict):
        if any(not isinstance(key, str) for key in value):
            raise TypeError(f"{path} must have string keys")
        for key, item in value.items():
            _check_json(item, f"{path}.{key}")
        return
    raise TypeError(f"{path} contains a non-JSON value: {type(value).__name__}")


def canonical_json(value: Any) -> bytes:
    """Canonical UTF-8 JSON, preserving exact integers and rejecting NaN."""
    _check_json(value)
    return json.dumps(
        value, ensure_ascii=False, sort_keys=True, separators=(",", ":"), allow_nan=False
    ).encode("utf-8")


def stable_id(prefix: str, value: Any) -> str:
    """Derive a stable identity from semantic inputs, independent of run order."""
    if not prefix or any(char not in "abcdefghijklmnopqrstuvwxyz0123456789_" for char in prefix):
        raise ValueError("ID prefix must contain only lowercase letters, digits, or underscore")
    return f"{prefix}_{hashlib.sha256(canonical_json(value)).hexdigest()}"


def metric(value: int | float | None, *, reason: str | None = None) -> dict[str, Any]:
    """Represent a measured zero separately from unavailable and unsupported."""
    if value is None:
        if not reason:
            raise ValueError("missing metric requires a reason")
    elif reason is not None:
        raise ValueError("measured metric cannot have a missing reason")
    result = {"value": value, "reason": reason}
    _check_json(result)
    return result


def validate_observation(record: Mapping[str, Any]) -> None:
    """Fail early on malformed v2 rows before writing an append-only run log."""
    if record.get("schema_version") != SCHEMA_VERSION_V2:
        raise ValueError("shortest-path observation has the wrong schema version")
    for name in ("run_id", "workload_id", "variant_id", "campaign_id", "pass_name"):
        if not isinstance(record.get(name), str) or not record[name]:
            raise ValueError(f"observation requires a non-empty {name}")
    for namespace in NAMESPACES:
        if not isinstance(record.get(namespace), dict):
            raise ValueError(f"observation requires {namespace} namespace")
    _check_json(dict(record))


def append_jsonl(path: str | Path, record: Mapping[str, Any], *, durable: bool = True) -> None:
    """Append one complete record; optionally sync for per-invocation recovery."""
    data = canonical_json(dict(record)) + b"\n"
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    with target.open("ab") as output:
        output.write(data)
        output.flush()
        if durable:
            os.fsync(output.fileno())


def recover_jsonl_tail(path: str | Path) -> bool:
    """Discard only a physically incomplete last line; retain completed rows.

    Returns whether a truncated tail was removed. A valid final JSON record
    without a newline is completed in place, so resume will not rerun it.
    """
    target = Path(path)
    if not target.exists():
        return False
    with target.open("r+b") as output:
        output.seek(0, os.SEEK_END)
        end = output.tell()
        if end == 0:
            return False
        output.seek(end - 1)
        if output.read(1) == b"\n":
            return False
        start = end - 1
        while start >= 0:
            output.seek(start)
            if output.read(1) == b"\n":
                break
            start -= 1
        output.seek(start + 1)
        tail = output.read(end - start - 1)
        try:
            json.loads(tail)
        except (UnicodeDecodeError, json.JSONDecodeError):
            output.truncate(start + 1)
            output.flush()
            os.fsync(output.fileno())
            return True
        output.seek(end)
        output.write(b"\n")
        output.flush()
        os.fsync(output.fileno())
        return False


def read_jsonl(path: str | Path) -> list[dict[str, Any]]:
    """Read completed rows, rejecting malformed interior lines."""
    target = Path(path)
    if not target.exists():
        return []
    records: list[dict[str, Any]] = []
    with target.open("r", encoding="utf-8") as source:
        for line_number, line in enumerate(source, start=1):
            try:
                record = json.loads(line)
            except json.JSONDecodeError as exc:
                raise ValueError(f"malformed JSONL at {target}:{line_number}") from exc
            if not isinstance(record, dict):
                raise ValueError(f"JSONL row is not an object at {target}:{line_number}")
            records.append(record)
    return records


def write_json_atomic(path: str | Path, value: Any) -> None:
    """Atomically replace a JSON artifact after its complete bytes are written."""
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    data = canonical_json(value) + b"\n"
    with tempfile.NamedTemporaryFile(mode="wb", dir=target.parent, delete=False) as output:
        temp_path = Path(output.name)
        try:
            output.write(data)
            output.flush()
            os.fsync(output.fileno())
        except BaseException:
            temp_path.unlink(missing_ok=True)
            raise
    os.replace(temp_path, target)


def create_report_v2(
    *,
    manifest: Mapping[str, Any],
    summary: Mapping[str, Any],
    output_path: str | Path | None = None,
) -> dict[str, Any]:
    """Wrap evaluated results with source and host provenance."""
    report = {
        "schema_version": SCHEMA_VERSION_V2,
        "created_at": utc_now(),
        "manifest": dict(manifest),
        "summary": dict(summary),
        "source": source_provenance(output_path),
        "environment": environment_metadata(),
    }
    _check_json(report)
    return report
