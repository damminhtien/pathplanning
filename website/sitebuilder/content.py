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
from .renderers.application_visuals import APPLICATION_VISUAL_IDS


@dataclass(frozen=True)
class SiteContent:
    planners: list[dict[str, Any]]
    lessons: dict[str, dict[str, Any]]
    analyses: dict[str, dict[str, str]]
    references: dict[str, dict[str, Any]]
    registry_rows: dict[str, dict[str, str]]
    project_map: dict[str, Any]
    applications: dict[str, Any]
    datasets: list[dict[str, Any]]
    case_studies: list[dict[str, Any]]
    dataset_snapshot: dict[str, Any]


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
    applications = read_json(WEBSITE_ROOT / "data" / "applications.json")
    dataset_catalog = read_json(WEBSITE_ROOT / "data" / "datasets.json")
    case_studies = read_json(WEBSITE_ROOT / "data" / "case_studies.json")
    if (
        not isinstance(planners, list)
        or not isinstance(lessons, dict)
        or not isinstance(analyses, dict)
        or not isinstance(references, dict)
        or not isinstance(project_map, dict)
        or not isinstance(applications, dict)
        or not isinstance(dataset_catalog, dict)
        or not isinstance(dataset_catalog.get("datasets"), list)
        or not isinstance(dataset_catalog.get("snapshot"), dict)
        or not isinstance(case_studies, list)
    ):
        raise SystemExit("Planner, learning, project-map, or dataset content has invalid types.")

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

    datasets = dataset_catalog["datasets"]
    if not datasets or any(not isinstance(dataset, dict) for dataset in datasets):
        raise SystemExit("The dataset catalog must contain dataset objects.")
    dataset_ids = [dataset.get("id") for dataset in datasets]
    if any(not isinstance(dataset_id, str) for dataset_id in dataset_ids):
        raise SystemExit("The dataset catalog must contain string IDs.")
    if len(dataset_ids) != len(set(dataset_ids)):
        raise SystemExit("The dataset catalog must contain unique string IDs.")
    dataset_id_set = set(dataset_ids)

    case_study_ids = [study.get("id") for study in case_studies if isinstance(study, dict)]
    if (
        len(case_study_ids) != len(case_studies)
        or not case_study_ids
        or any(not isinstance(study_id, str) for study_id in case_study_ids)
        or len(case_study_ids) != len(set(case_study_ids))
    ):
        raise SystemExit("Case studies must contain unique objects with string IDs.")
    for study in case_studies:
        study_id = study.get("id")
        if not isinstance(study_id, str) or not re.fullmatch(r"[a-z0-9]+(?:-[a-z0-9]+)*", study_id):
            raise SystemExit(f"Invalid case-study ID: {study_id!r}")
        required_fields = (
            "title",
            "eyebrow",
            "application",
            "summary",
            "question",
            "protocol",
            "interpretation",
            "limitation",
        )
        if not all(
            isinstance(study.get(field), str) and study[field].strip() for field in required_fields
        ):
            raise SystemExit(f"Case-study content is incomplete for {study_id}.")
        if study.get("dataset_id") not in dataset_id_set:
            raise SystemExit(f"Case study {study_id} references an unknown dataset.")
        if study.get("diagram") not in {"grid", "road", "continuous", "voxel"}:
            raise SystemExit(f"Case study {study_id} has an unknown diagram.")
        for field, minimum in (("stats", 2), ("findings", 3), ("sources", 1)):
            records = study.get(field)
            if not isinstance(records, list) or len(records) < minimum:
                raise SystemExit(f"Case study {study_id} needs at least {minimum} {field}.")
            required_record_fields = {
                "stats": ("value", "label"),
                "findings": ("title", "text"),
                "sources": ("label", "url"),
            }[field]
            for record in records:
                if not isinstance(record, dict) or not all(
                    isinstance(record.get(key), str) and record[key].strip()
                    for key in required_record_fields
                ):
                    raise SystemExit(f"Case-study {field} are incomplete for {study_id}.")
                if field == "sources" and not record["url"].startswith("https://"):
                    raise SystemExit(f"Case-study source URL must use HTTPS for {study_id}.")
    case_study_words = len(
        re.findall(
            r"\b[\w'-]+\b",
            " ".join(
                text
                for study in case_studies
                for text in [
                    study["title"],
                    study["summary"],
                    study["question"],
                    study["protocol"],
                    study["interpretation"],
                    study["limitation"],
                    *(finding[key] for finding in study["findings"] for key in ("title", "text")),
                ]
            ),
        )
    )
    if case_study_words < 600:
        raise SystemExit("The case-study collection needs more detailed analysis.")

    application_overview = applications.get("overview")
    application_families = applications.get("families")
    if (
        not isinstance(application_overview, list)
        or len(application_overview) < 6
        or not isinstance(application_families, dict)
        or set(application_families) != set(FAMILY_LABELS)
    ):
        raise SystemExit("Application coverage must include an overview and every planner family.")

    planner_id_set = set(ids)

    def validate_application_link(link: Any) -> None:
        match = (
            re.fullmatch(r"(datasets|algorithms)/([a-z0-9]+(?:[_-][a-z0-9]+)*)/", link)
            if isinstance(link, str)
            else None
        )
        if not match:
            raise SystemExit(f"Invalid application link: {link!r}")
        route_type, route_id = match.groups()
        known_ids = dataset_id_set if route_type == "datasets" else planner_id_set
        if route_id not in known_ids:
            raise SystemExit(f"Application link points to an unknown {route_type} page: {link}")

    for application in application_overview:
        required_fields = (
            "category",
            "title",
            "description",
            "project_note",
            "link_label",
            "visual",
        )
        if not isinstance(application, dict) or not all(
            isinstance(application.get(field), str) and application[field].strip()
            for field in required_fields
        ):
            raise SystemExit("Application overview entries are incomplete.")
        if application["visual"] not in APPLICATION_VISUAL_IDS:
            raise SystemExit(f"Unknown application visual: {application['visual']}")
        validate_application_link(application.get("link"))

    for kind, family_applications in application_families.items():
        if not isinstance(family_applications, list) or len(family_applications) < 3:
            raise SystemExit(f"At least three application contexts are required for {kind}.")
        for application in family_applications:
            required_fields = ("title", "scenario", "connection")
            if not isinstance(application, dict) or not all(
                isinstance(application.get(field), str) and application[field].strip()
                for field in required_fields
            ):
                raise SystemExit(f"An application context is incomplete for {kind}.")

    for dataset in datasets:
        dataset_id = dataset["id"]
        required_dataset_fields = (
            "title",
            "eyebrow",
            "tagline",
            "description",
            "source_name",
            "inventory_scope",
            "campaign_status",
            "source_url",
        )
        if not re.fullmatch(r"[a-z0-9]+(?:-[a-z0-9]+)*", dataset_id):
            raise SystemExit(f"Invalid dataset page ID: {dataset_id!r}")
        if not all(
            isinstance(dataset.get(field), str) and dataset[field].strip()
            for field in required_dataset_fields
        ):
            raise SystemExit(f"Dataset metadata is incomplete for {dataset_id}.")
        if dataset.get("schema_type") not in {"Dataset", "TechArticle"}:
            raise SystemExit(f"Unsupported structured-data type for {dataset_id}.")
        if not dataset["source_url"].startswith("https://"):
            raise SystemExit(f"Dataset source URL must use HTTPS for {dataset_id}.")

        stats = dataset.get("stats")
        if (
            not isinstance(stats, list)
            or len(stats) < 3
            or any(
                not isinstance(row, dict)
                or not isinstance(row.get("value"), str)
                or not row["value"].strip()
                or not isinstance(row.get("label"), str)
                or not row["label"].strip()
                for row in stats
            )
        ):
            raise SystemExit(f"Dataset statistics are incomplete for {dataset_id}.")

        sections = dataset.get("sections")
        if not isinstance(sections, list):
            raise SystemExit(f"Dataset sections are invalid for {dataset_id}.")
        section_ids = [
            section.get("id") if isinstance(section, dict) else None for section in sections
        ]
        if (
            len(sections) < 4
            or len(section_ids) != len(set(section_ids))
            or any(not isinstance(section_id, str) for section_id in section_ids)
        ):
            raise SystemExit(f"Dataset sections are incomplete for {dataset_id}.")
        for section in sections:
            if not all(
                isinstance(section.get(field), str) and section[field].strip()
                for field in ("id", "label", "title")
            ):
                raise SystemExit(f"Dataset section heading is incomplete for {dataset_id}.")
            paragraphs = section.get("paragraphs", [])
            bullets = section.get("bullets", [])
            if not isinstance(paragraphs, list) or not isinstance(bullets, list):
                raise SystemExit(f"Dataset section copy is invalid for {dataset_id}.")
            if any(
                not isinstance(text, str) or not text.strip() for text in [*paragraphs, *bullets]
            ):
                raise SystemExit(f"Dataset section copy is incomplete for {dataset_id}.")
            table = section.get("table")
            if table:
                if (
                    not isinstance(table, dict)
                    or not isinstance(table.get("caption"), str)
                    or not table["caption"].strip()
                    or not isinstance(table.get("headers"), list)
                    or not table["headers"]
                    or not isinstance(table.get("rows"), list)
                    or any(
                        not isinstance(cell, str) or not cell.strip() for cell in table["headers"]
                    )
                    or any(
                        not isinstance(row, list)
                        or len(row) != len(table["headers"])
                        or any(not isinstance(cell, str) or not cell.strip() for cell in row)
                        for row in table["rows"]
                    )
                ):
                    raise SystemExit(f"Dataset table is invalid for {dataset_id}.")
            callout = section.get("callout")
            if callout:
                if not isinstance(callout, dict) or not all(
                    isinstance(callout.get(field), str) and callout[field].strip()
                    for field in ("title", "text")
                ):
                    raise SystemExit(f"Dataset callout is invalid for {dataset_id}.")
            if not paragraphs and not bullets and not section.get("table"):
                raise SystemExit(f"Dataset section has no content for {dataset_id}.")
        word_count = len(
            re.findall(
                r"\b[\w'-]+\b",
                " ".join(
                    text
                    for section in sections
                    for text in [
                        section["title"],
                        *section.get("paragraphs", []),
                        *section.get("bullets", []),
                    ]
                ),
            )
        )
        if word_count < 300:
            raise SystemExit(f"Dataset page content is too short for {dataset_id}.")

        sources = dataset.get("sources")
        if not isinstance(sources, list) or not sources:
            raise SystemExit(f"Dataset sources are missing for {dataset_id}.")
        for source in sources:
            if (
                not isinstance(source, dict)
                or not all(
                    isinstance(source.get(field), str) and source[field].strip()
                    for field in ("name", "url", "context")
                )
                or not source["url"].startswith("https://")
            ):
                raise SystemExit(f"Dataset source reference is incomplete for {dataset_id}.")

        project_paths = dataset.get("project_paths", [])
        if not isinstance(project_paths, list) or not project_paths:
            raise SystemExit(f"Dataset project references are missing for {dataset_id}.")
        for path in project_paths:
            if not isinstance(path, str) or not (PROJECT_ROOT / path).is_file():
                raise SystemExit(f"Dataset project reference does not exist: {path}")
        related = dataset.get("related", [])
        if (
            not isinstance(related, list)
            or any(not isinstance(related_id, str) for related_id in related)
            or not set(related).issubset(dataset_id_set)
        ):
            raise SystemExit(f"Dataset page {dataset_id} references an unknown dataset.")

    snapshot = dataset_catalog["snapshot"]
    for field in (
        "inventory_date",
        "evaluation_date",
        "characterization_path",
        "installation_path",
    ):
        if not isinstance(snapshot.get(field), str) or not snapshot[field].strip():
            raise SystemExit(f"Dataset snapshot is missing {field}.")
    for field in (
        "entry_count",
        "collection_count",
        "source_count",
        "observation_count",
        "cohort_count",
    ):
        if type(snapshot.get(field)) is not int or snapshot[field] <= 0:
            raise SystemExit(f"Dataset snapshot has an invalid {field}.")
    for field in ("characterization_path", "installation_path"):
        if not (PROJECT_ROOT / snapshot[field]).is_file():
            raise SystemExit(f"Dataset snapshot document does not exist: {snapshot[field]}")

    return SiteContent(
        planners=planners,
        lessons=lessons,
        analyses=analyses,
        references=references,
        registry_rows=rows,
        project_map=project_map,
        applications=applications,
        datasets=datasets,
        case_studies=case_studies,
        dataset_snapshot=snapshot,
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
