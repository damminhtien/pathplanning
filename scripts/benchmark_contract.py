"""Shared, versioned output contract for repository benchmark scripts."""

from __future__ import annotations

from collections import defaultdict
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import shlex
import shutil
import statistics
import subprocess
import sys
import sysconfig
import tempfile
import time
from typing import Any, Callable
import uuid

import numpy as np

SCHEMA_VERSION = "pathplanning_benchmark_v1"
_REPOSITORY_ROOT = Path(__file__).resolve().parents[1]


def utc_now() -> str:
    """Return a timezone-aware UTC timestamp in a JSON-friendly form."""
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")


def _canonical_json(value: Any) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()


def _json_safe(value: Any) -> Any:
    if isinstance(value, np.generic):
        return _json_safe(value.item())
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    return value


def _sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _relative_path(path: Path) -> str:
    try:
        return path.resolve().relative_to(_REPOSITORY_ROOT).as_posix()
    except ValueError:
        return str(path.resolve())


def _excluded_paths() -> list[str]:
    return ["graphify-out/**", "benchmark-results/**"]


def _output_path_relative(output_path: str | Path | None) -> str | None:
    if output_path is None:
        return None
    path = Path(output_path).expanduser()
    if not path.is_absolute():
        path = Path.cwd() / path
    try:
        return path.resolve().relative_to(_REPOSITORY_ROOT).as_posix()
    except ValueError:
        return None


def source_provenance(output_path: str | Path | None = None) -> dict[str, Any]:
    """Fingerprint committed and local source inputs, excluding generated output."""
    excluded = _excluded_paths()
    output_relative = _output_path_relative(output_path)
    path_exclusions = [*excluded]
    if output_relative is not None:
        path_exclusions.append(output_relative)
    try:
        commit = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=_REPOSITORY_ROOT,
            check=True,
            capture_output=True,
            text=True,
            timeout=5,
        ).stdout.strip()
        branch_result = subprocess.run(
            ["git", "branch", "--show-current"],
            cwd=_REPOSITORY_ROOT,
            check=True,
            capture_output=True,
            text=True,
            timeout=5,
        )
        pathspecs = [".", *(f":(exclude){path}" for path in path_exclusions)]
        diff = subprocess.run(
            ["git", "diff", "--binary", "HEAD", "--", *pathspecs],
            cwd=_REPOSITORY_ROOT,
            check=True,
            capture_output=True,
            timeout=10,
        ).stdout
        untracked_result = subprocess.run(
            ["git", "ls-files", "--others", "--exclude-standard", "-z"],
            cwd=_REPOSITORY_ROOT,
            check=True,
            capture_output=True,
            timeout=5,
        )
        excluded_paths = [Path(path).as_posix() for path in excluded]
        untracked_files = []
        for raw_path in untracked_result.stdout.split(b"\0"):
            if not raw_path:
                continue
            relative = Path(os.fsdecode(raw_path)).as_posix()
            if output_relative is not None and relative == output_relative:
                continue
            if any(
                (relative.startswith(path[:-2]) if path.endswith("/**") else relative == path)
                for path in excluded_paths
            ):
                continue
            path = _REPOSITORY_ROOT / relative
            if path.is_file():
                untracked_files.append({"path": relative, "sha256": _sha256_file(path)})

        untracked_files.sort(key=lambda item: item["path"])
        worktree_fingerprint = _sha256_bytes(diff + _canonical_json(untracked_files))
        return {
            "available": True,
            "commit": commit,
            "branch": branch_result.stdout.strip() or None,
            "dirty": bool(diff or untracked_files),
            "worktree_fingerprint": worktree_fingerprint,
            "excluded_from_fingerprint": excluded,
            "untracked_files": untracked_files,
        }
    except (OSError, subprocess.SubprocessError) as exc:
        return {
            "available": False,
            "commit": None,
            "branch": None,
            "dirty": None,
            "worktree_fingerprint": None,
            "excluded_from_fingerprint": excluded,
            "error": f"{type(exc).__name__}: {exc}",
        }


def _compiler_info(env_name: str, config_key: str) -> dict[str, str | None]:
    configured = os.environ.get(env_name) or str(sysconfig.get_config_var(config_key) or "")
    pieces = shlex.split(configured)
    executable = shutil.which(pieces[0]) if pieces else None
    version = None
    if executable is not None:
        try:
            completed = subprocess.run(
                [executable, "--version"],
                check=False,
                capture_output=True,
                text=True,
                timeout=5,
            )
            output = completed.stdout or completed.stderr
            version = output.splitlines()[0].strip() if output else None
        except (OSError, subprocess.TimeoutExpired):
            pass
    return {"configured": configured or None, "executable": executable, "version": version}


def _processor_name() -> str | None:
    if platform.system() == "Darwin":
        try:
            completed = subprocess.run(
                ["sysctl", "-n", "machdep.cpu.brand_string"],
                check=False,
                capture_output=True,
                text=True,
                timeout=2,
            )
            model = completed.stdout.strip()
            if completed.returncode == 0 and model:
                return model
        except (OSError, subprocess.TimeoutExpired):
            pass
    processor = platform.processor()
    if processor.lower() in {"", "i386", "i686"}:
        processor = platform.uname().machine
    return processor or None


def environment_metadata() -> dict[str, Any]:
    """Describe the runtime and configured native toolchain."""
    return {
        "platform": platform.platform(),
        "system": platform.system(),
        "machine": platform.machine(),
        "processor": _processor_name(),
        "logical_cpu_count": os.cpu_count(),
        "python": sys.version,
        "numpy": np.__version__,
        "c_compiler": _compiler_info("CC", "CC"),
        "cxx_compiler": _compiler_info("CXX", "CXX"),
        "native_compile_flags": {
            "c": ["-std=c11", "-O3"],
            "c++": ["-std=c++17", "-O3"],
            "cflags_environment": os.environ.get("CFLAGS"),
            "cxxflags_environment": os.environ.get("CXXFLAGS"),
        },
    }


def native_artifacts() -> list[dict[str, Any]]:
    """Return hashes of compiled search libraries present in the checkout."""
    native_directory = _REPOSITORY_ROOT / "pathplanning" / "native"
    artifacts = []
    for library_name in ("_search_engine", "_continuous_engine"):
        for suffix in (".so", ".dylib", ".dll", ".pyd"):
            for path in native_directory.glob(f"{library_name}*{suffix}"):
                if path.is_file():
                    artifacts.append(
                        {
                            "path": _relative_path(path),
                            "sha256": _sha256_file(path),
                            "size_bytes": path.stat().st_size,
                        }
                    )
    return sorted(artifacts, key=lambda item: item["path"])


def execute_run(
    *,
    case_id: str,
    variant_id: str,
    phase: str,
    repeat_index: int,
    seed: int,
    run: Callable[[], dict[str, Any]],
) -> dict[str, Any]:
    """Execute one workload and retain errors as explicit raw observations."""
    started_at = utc_now()
    started = time.perf_counter()
    try:
        observation = run()
        status = "completed"
        error = None
    except Exception as exc:  # Keep partial benchmark campaigns inspectable.
        observation = {}
        status = "error"
        error = {"type": type(exc).__name__, "message": str(exc)}
    elapsed_s = time.perf_counter() - started
    record = {
        "case_id": case_id,
        "variant_id": variant_id,
        "phase": phase,
        "repeat_index": repeat_index,
        "seed": seed,
        "started_at_utc": started_at,
        "finished_at_utc": utc_now(),
        "status": status,
        "elapsed_s": elapsed_s,
        "measurements": observation.get("measurements", {}),
        "outcomes": observation.get("outcomes", {}),
    }
    if "details" in observation:
        record["details"] = observation["details"]
    if error is not None:
        record["error"] = error
    return _json_safe(record)


def summarize_runs(runs: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """Summarize measured runs with explicit completion and success denominators."""
    grouped: dict[tuple[str, str], list[dict[str, Any]]] = defaultdict(list)
    for run in runs:
        if run["phase"] == "measure":
            grouped[(run["case_id"], run["variant_id"])].append(run)

    summaries = []
    for (case_id, variant_id), group in grouped.items():
        completed = [run for run in group if run["status"] == "completed"]
        success_count = sum(run.get("outcomes", {}).get("success") is True for run in completed)
        metric_values: dict[str, list[float | int]] = defaultdict(list)
        for run in completed:
            for name, value in run.get("measurements", {}).items():
                if isinstance(value, (int, float)) and not isinstance(value, bool):
                    if math.isfinite(float(value)):
                        metric_values[name].append(value)
        metrics = {}
        for name, values in sorted(metric_values.items()):
            metrics[name] = {
                "count": len(values),
                "median": statistics.median(values),
                "min": min(values),
                "max": max(values),
            }
        summaries.append(
            {
                "case_id": case_id,
                "variant_id": variant_id,
                "attempt_count": len(group),
                "completed_count": len(completed),
                "error_count": len(group) - len(completed),
                "success_count": success_count,
                "success_rate": success_count / len(completed) if completed else None,
                "success_rate_denominator": "completed_count",
                "metrics": metrics,
            }
        )
    return summaries


def create_report(
    *,
    benchmark_id: str,
    settings: dict[str, Any],
    workloads: list[dict[str, Any]],
    runs: list[dict[str, Any]],
    started_at_utc: str,
    started_monotonic: float,
    output_path: str | Path | None = None,
) -> dict[str, Any]:
    """Build a versioned report and stable experiment identity."""
    identity = {
        "benchmark_id": benchmark_id,
        "settings": settings,
        "workloads": workloads,
        "source": source_provenance(output_path),
        "environment": environment_metadata(),
        "native_artifacts": native_artifacts(),
    }
    return {
        "schema_version": SCHEMA_VERSION,
        "benchmark_id": benchmark_id,
        "run_id": str(uuid.uuid4()),
        "experiment_id": _sha256_bytes(_canonical_json(identity)),
        "started_at_utc": started_at_utc,
        "finished_at_utc": utc_now(),
        "duration_s": time.perf_counter() - started_monotonic,
        "status": (
            "completed_with_errors"
            if any(run["status"] == "error" for run in runs)
            else "completed"
        ),
        "command": list(sys.argv),
        "identity": identity,
        "runs": runs,
        "results": summarize_runs(runs),
    }


def write_report(path: str | Path, report: dict[str, Any]) -> None:
    """Write JSON atomically so interrupted runs do not leave partial reports."""
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary_path: str | None = None
    try:
        with tempfile.NamedTemporaryFile(
            "w",
            encoding="utf-8",
            dir=destination.parent,
            prefix=f".{destination.name}.",
            suffix=".tmp",
            delete=False,
        ) as output:
            temporary_path = output.name
            json.dump(report, output, indent=2, sort_keys=True, allow_nan=False)
            output.write("\n")
            output.flush()
            os.fsync(output.fileno())
        os.replace(temporary_path, destination)
    except BaseException:
        if temporary_path is not None:
            Path(temporary_path).unlink(missing_ok=True)
        raise


__all__ = [
    "SCHEMA_VERSION",
    "create_report",
    "environment_metadata",
    "execute_run",
    "native_artifacts",
    "utc_now",
    "write_report",
]
