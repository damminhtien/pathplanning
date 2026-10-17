"""Load and validate the source data used to generate the site."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
import re
from typing import Any

from .config import (
    FAMILY_LABELS,
    FAMILY_SUMMARIES,
    PLANNER_ROWS,
    PROJECT_ROOT,
    WEBSITE_ROOT,
)


@dataclass(frozen=True)
class SiteContent:
    planners: list[dict[str, Any]]
    lessons: dict[str, dict[str, Any]]
    analyses: dict[str, dict[str, str]]
    references: dict[str, dict[str, Any]]
    registry_rows: dict[str, dict[str, str]]
    project_map: dict[str, Any]


def read_json(path: Path) -> Any:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise SystemExit(f"Cannot read {path.relative_to(PROJECT_ROOT)}: {exc}") from exc


def load_site_content() -> SiteContent:
    planners = read_json(WEBSITE_ROOT / "data" / "planners.json")
    lessons = read_json(WEBSITE_ROOT / "data" / "tutorials.json")
    analyses = read_json(WEBSITE_ROOT / "data" / "algorithm_analysis.json")
    references = read_json(WEBSITE_ROOT / "data" / "references.json")
    project_map = read_json(WEBSITE_ROOT / "data" / "project-map.json")
    if (
        not isinstance(planners, list)
        or not isinstance(lessons, dict)
        or not isinstance(analyses, dict)
        or not isinstance(references, dict)
        or not isinstance(project_map, dict)
    ):
        raise SystemExit(
            "Planner lesson, analysis, reference, or project-map data has invalid types."
        )

    matrix = (PROJECT_ROOT / "SUPPORTED_ALGORITHMS.md").read_text(encoding="utf-8")
    rows = {match["id"]: match.groupdict() for match in PLANNER_ROWS.finditer(matrix)}
    ids = [planner.get("id") for planner in planners]
    if len(ids) != len(set(ids)):
        raise SystemExit("The planner catalogue contains duplicate IDs.")
    if set(ids) != set(rows):
        missing = sorted(set(rows) - set(ids))
        extra = sorted(set(ids) - set(rows))
        raise SystemExit(
            f"Planner catalogue differs from SUPPORTED_ALGORITHMS.md: "
            f"missing={missing}, extra={extra}"
        )
    if set(ids) != set(lessons):
        missing = sorted(set(ids) - set(lessons))
        extra = sorted(set(lessons) - set(ids))
        raise SystemExit(
            f"Tutorial coverage differs from the planner catalogue: "
            f"missing={missing}, extra={extra}"
        )
    if set(ids) != set(analyses):
        missing = sorted(set(ids) - set(analyses))
        extra = sorted(set(analyses) - set(ids))
        raise SystemExit(
            f"Algorithm-analysis coverage differs from the planner catalogue: "
            f"missing={missing}, extra={extra}"
        )
    if set(ids) != set(references):
        missing = sorted(set(ids) - set(references))
        extra = sorted(set(references) - set(ids))
        raise SystemExit(
            f"Paper-reference coverage differs from the planner catalogue: "
            f"missing={missing}, extra={extra}"
        )

    unknown_kinds = sorted({planner.get("kind") for planner in planners} - set(FAMILY_LABELS))
    if unknown_kinds:
        raise SystemExit(f"Unknown planner families: {unknown_kinds}")

    for planner in planners:
        planner_id = planner["id"]
        lesson = lessons[planner_id]
        analysis = analyses[planner_id]
        reference = references[planner_id]
        required_reference_fields = ("authors", "title", "venue", "label", "url")
        if not isinstance(reference, dict) or not all(
            isinstance(reference.get(field), str) and reference[field].strip()
            for field in required_reference_fields
        ):
            raise SystemExit(f"Paper reference is incomplete for {planner_id}.")
        if type(reference.get("year")) is not int or not 1900 <= reference["year"] <= 2100:
            raise SystemExit(f"Paper reference has an invalid year for {planner_id}.")
        if not reference["url"].startswith("https://"):
            raise SystemExit(f"Paper reference URL must use HTTPS for {planner_id}.")
        if not all(
            lesson.get(field) for field in ("tagline", "story", "experiment", "question", "answer")
        ):
            raise SystemExit(f"Tutorial content is incomplete for {planner_id}.")
        if len(lesson.get("steps", [])) != 3:
            raise SystemExit(f"{planner_id} must have exactly three teaching steps.")
        example = lesson.get("example", {})
        required_lesson_fields = ("mechanism", "best_for", "watch_out", "evaluation")
        if not all(lesson.get(field) for field in required_lesson_fields):
            raise SystemExit(f"Expanded lesson content is incomplete for {planner_id}.")
        if not example.get("setup") or len(example.get("steps", [])) != 3:
            raise SystemExit(f"{planner_id} must have a setup and three worked-example steps.")
        if len(lesson.get("metrics", [])) < 3:
            raise SystemExit(f"{planner_id} must name at least three evaluation metrics.")
        required_analysis_fields = (
            "time_order",
            "space_order",
            "model",
            "time",
            "space",
            "guarantee",
            "scaling",
        )
        if not all(analysis.get(field) for field in required_analysis_fields):
            raise SystemExit(f"Algorithm analysis is incomplete for {planner_id}.")
        analysis_copy = " ".join(
            analysis[field] for field in ("model", "time", "space", "guarantee", "scaling")
        )
        if len(re.findall(r"\b[\w*·×−≥/+-]+\b", analysis_copy)) < 100:
            raise SystemExit(f"Algorithm analysis is too short for {planner_id}.")
        expanded_copy = " ".join(
            [
                lesson["mechanism"],
                lesson["best_for"],
                lesson["watch_out"],
                lesson["evaluation"],
                example["setup"],
                *example["steps"],
            ]
        )
        if len(re.findall(r"\b[\w*'-]+\b", expanded_copy)) < 150:
            raise SystemExit(f"Expanded tutorial content is too short for {planner_id}.")
        if not set(lesson.get("related", [])).issubset(set(ids)):
            raise SystemExit(f"Tutorial for {planner_id} references an unknown planner.")
        module = rows[planner_id]["module"].replace(".", "/") + ".py"
        if not (PROJECT_ROOT / module).is_file():
            raise SystemExit(f"Source module for {planner_id} does not exist: {module}")

    def validate_map_paths(node: dict[str, Any]) -> None:
        path = node.get("path")
        if path and not (PROJECT_ROOT / path).exists():
            raise SystemExit(f"Project map source does not exist: {path}")
        for child in node.get("children", []):
            validate_map_paths(child)

    validate_map_paths(project_map)

    return SiteContent(
        planners=planners,
        lessons=lessons,
        analyses=analyses,
        references=references,
        registry_rows=rows,
        project_map=project_map,
    )


def source_url(path: str) -> str:
    target = PROJECT_ROOT / path
    endpoint = "tree" if target.is_dir() else "blob"
    return f"https://github.com/damminhtien/pathplanning/{endpoint}/main/{path}"


def expand_project_map(
    node: dict[str, Any],
    planners: list[dict[str, Any]],
    registry_rows: dict[str, dict[str, str]],
) -> dict[str, Any]:
    expanded = dict(node)
    if node.get("childrenFrom") == "plannerKinds":
        expanded["label"] = f"Planner atlas, {len(planners)} algorithms"
        expanded["children"] = []
        for kind, label in FAMILY_LABELS.items():
            family = {
                "id": f"family-{kind}",
                "label": label,
                "type": "family",
                "summary": FAMILY_SUMMARIES[kind],
                "path": "SUPPORTED_ALGORITHMS.md",
                "children": [],
            }
            for planner in planners:
                if planner["kind"] == kind:
                    module = registry_rows[planner["id"]]["module"].replace(".", "/") + ".py"
                    family["children"].append(
                        {
                            "id": f"planner-{planner['id']}",
                            "label": planner["title"],
                            "type": "planner",
                            "summary": planner["description"],
                            "path": module,
                            "page": f"/algorithms/{planner['id']}/",
                            "principle": planner["principle"],
                        }
                    )
            expanded["children"].append(family)
    elif node.get("children"):
        expanded["children"] = [
            expand_project_map(child, planners, registry_rows) for child in node["children"]
        ]
    return expanded
