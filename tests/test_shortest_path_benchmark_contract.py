"""Exact counts, unavailable values, and interruption recovery in v2 files."""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from scripts.benchmark_contract import SCHEMA_VERSION
from scripts.shortest_path_benchmark.contract import (
    SCHEMA_VERSION_V2,
    append_jsonl,
    canonical_json,
    metric,
    read_jsonl,
    recover_jsonl_tail,
    stable_id,
    validate_observation,
    write_json_atomic,
)


def test_v2_exact_large_integer_zero_and_missing_reason() -> None:
    huge = 2**53 + 101
    value = {"work": {"frontier_pushes": huge, "closed_skips": 0}}
    assert json.loads(canonical_json(value))["work"]["frontier_pushes"] == huge
    assert metric(0) == {"value": 0, "reason": None}
    assert metric(None, reason="unsupported") == {"value": None, "reason": "unsupported"}
    with pytest.raises(ValueError, match="reason"):
        metric(None)
    with pytest.raises(ValueError, match="finite"):
        canonical_json({"value": float("nan")})
    assert SCHEMA_VERSION == "pathplanning_benchmark_v1"
    assert SCHEMA_VERSION_V2 != SCHEMA_VERSION


def test_observation_validation_and_stable_identity() -> None:
    payload = {"map": "sha", "start": 1, "goal": 4}
    assert stable_id("workload", payload) == stable_id(
        "workload", dict(reversed(list(payload.items())))
    )
    record = {
        "schema_version": SCHEMA_VERSION_V2,
        "run_id": "run_1",
        "workload_id": "w_1",
        "variant_id": "v_1",
        "campaign_id": "c_1",
        "pass_name": "work",
        "input": {},
        "outcome": {},
        "work": {"expanded": 2**53 + 1},
        "memory": {},
        "timing": {},
        "capabilities": {},
        "provenance": {},
    }
    validate_observation(record)
    with pytest.raises(ValueError, match="namespace"):
        validate_observation({key: value for key, value in record.items() if key != "work"})


def test_jsonl_tail_recovery_and_atomic_snapshot(tmp_path: Path) -> None:
    output = tmp_path / "runs.jsonl"
    append_jsonl(output, {"run_id": "a", "counter": 2**53 + 1})
    with output.open("ab") as sink:
        sink.write(b'{"run_id":"partial"')
    assert recover_jsonl_tail(output)
    assert read_jsonl(output) == [{"counter": 2**53 + 1, "run_id": "a"}]
    with output.open("ab") as sink:
        sink.write(b'{"run_id":"complete"}')
    assert not recover_jsonl_tail(output)
    assert len(read_jsonl(output)) == 2
    write_json_atomic(tmp_path / "summary.json", {"expected": 2, "complete": 2})
    assert json.loads((tmp_path / "summary.json").read_text())["complete"] == 2
    write_json_atomic(tmp_path / "schedule.json", ["first", "second"])
    assert json.loads((tmp_path / "schedule.json").read_text()) == ["first", "second"]
