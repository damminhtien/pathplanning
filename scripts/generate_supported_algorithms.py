"""Generate the supported planner matrix from the planner registry."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
DOCUMENT = ROOT / "SUPPORTED_ALGORITHMS.md"
BEGIN_MARKER = "<!-- BEGIN GENERATED PLANNER MATRIX -->"
END_MARKER = "<!-- END GENERATED PLANNER MATRIX -->"


def render_matrix() -> str:
    """Render planner rows and constraints from the canonical registry."""
    root_text = str(ROOT)
    if root_text not in sys.path:
        sys.path.insert(0, root_text)

    from pathplanning.registry import PLANNER_REGISTRY

    lines = [
        "| Problem Kind | Planner | Module | Callable | Status |",
        "| ------------ | ------- | ------ | -------- | ------ |",
    ]
    constraints: dict[str, list[str]] = {}
    for name, spec in PLANNER_REGISTRY.items():
        module = getattr(spec.planner, "__module__")
        callable_name = getattr(spec.planner, "__name__")
        lines.append(
            f"| `{spec.problem_kind}` | `{name}` | `{module}` | `{callable_name}` | supported |"
        )
        for constraint in spec.constraints:
            constraints.setdefault(constraint, []).append(name)

    if constraints:
        lines.extend(("", "Planner-specific constraints:", ""))
        for constraint, names in constraints.items():
            planner_names = ", ".join(f"`{name}`" for name in names)
            lines.append(f"- {planner_names}: {constraint}")

    return "\n".join(lines)


def update_document(document: str, generated: str) -> str:
    """Replace the generated section while preserving the rest of the page."""
    if document.count(BEGIN_MARKER) != 1 or document.count(END_MARKER) != 1:
        raise ValueError("Expected exactly one pair of generated planner matrix markers")

    start = document.index(BEGIN_MARKER) + len(BEGIN_MARKER)
    end = document.index(END_MARKER, start)
    if end < start:
        raise ValueError("Generated planner matrix markers are out of order")
    return f"{document[:start]}\n{generated}\n{document[end:]}"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--check",
        action="store_true",
        help="exit with an error if the document does not match the registry",
    )
    args = parser.parse_args()

    current = DOCUMENT.read_text(encoding="utf-8")
    updated = update_document(current, render_matrix())
    if args.check:
        if current != updated:
            print(
                "SUPPORTED_ALGORITHMS.md is out of date; run this script to update it.",
                file=sys.stderr,
            )
            return 1
        return 0

    DOCUMENT.write_text(updated, encoding="utf-8")
    print(f"Updated {DOCUMENT.relative_to(ROOT)} from pathplanning.registry")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
