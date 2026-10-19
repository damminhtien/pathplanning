"""Check that repository XML and SVG assets are well formed."""

from __future__ import annotations

from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def repository_files() -> list[Path]:
    result = subprocess.run(
        ["git", "ls-files", "-z", "-c", "-o", "--exclude-standard"],
        cwd=ROOT,
        check=True,
        capture_output=True,
    )
    return sorted(ROOT / name.decode("utf-8") for name in result.stdout.split(b"\0") if name)


def main() -> int:
    files = [path for path in repository_files() if path.suffix.lower() in {".svg", ".xml"}]
    failures: list[str] = []
    for path in files:
        try:
            ET.parse(path)
        except (ET.ParseError, OSError) as error:
            failures.append(f"{path.relative_to(ROOT)}: {error}")

    if failures:
        print("XML/SVG validation failed:\n" + "\n".join(failures), file=sys.stderr)
        return 1

    print(f"XML/SVG validation passed: {len(files)} files.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
