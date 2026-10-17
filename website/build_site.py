#!/usr/bin/env python3
"""Build GitHub Pages and one static, indexable lesson per supported planner."""

from __future__ import annotations

import argparse
import collections
import datetime as dt
import html
import json
import re
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any


WEBSITE_ROOT = Path(__file__).resolve().parent
PROJECT_ROOT = WEBSITE_ROOT.parent
BASE_URL = "https://damminhtien.github.io/pathplanning"
BUILD_MARKER = ".pathplanning-site-build"
PLANNER_ROWS = re.compile(
    r"^\| \x60(?P<kind>[^\x60]+)\x60 \| \x60(?P<id>[^\x60]+)\x60 \| "
    r"\x60(?P<module>[^\x60]+)\x60 \| \x60(?P<callable>[^\x60]+)\x60 \|",
    re.MULTILINE,
)
EXPECTED_FAMILY_COUNTS = {
    "discrete": 15,
    "continuous": 17,
    "temporal": 3,
    "multi_agent": 2,
}
FAMILY_LABELS = {
    "discrete": "Grid & graph",
    "continuous": "Continuous space",
    "temporal": "Time-aware",
    "multi_agent": "Multi-agent",
}
FAMILY_SUMMARIES = {
    "discrete": "Choose how the frontier grows across a graph: by layers, cost, a heuristic, a repaired route, or visible shortcuts.",
    "continuous": "Search a continuous robot space by sampling, connecting, improving, batching, or following kinematic motion rules.",
    "temporal": "Plan through safe time windows, with waiting, duration, velocity, and acceleration rules made explicit.",
    "multi_agent": "Coordinate several agents while resolving vertex and edge-swap conflicts on a shared graph.",
}


def read_json(path: Path) -> Any:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise SystemExit(f"Cannot read {path.relative_to(PROJECT_ROOT)}: {exc}") from exc


def load_catalog() -> tuple[list[dict[str, Any]], dict[str, dict[str, str]], dict[str, Any]]:
    planners = read_json(WEBSITE_ROOT / "data" / "planners.json")
    lessons = read_json(WEBSITE_ROOT / "data" / "tutorials.json")
    project_map = read_json(WEBSITE_ROOT / "data" / "project-map.json")
    if not isinstance(planners, list) or not isinstance(lessons, dict):
        raise SystemExit("Planner and lesson data must be a list and an object.")

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

    counts = collections.Counter(planner.get("kind") for planner in planners)
    if counts != collections.Counter(EXPECTED_FAMILY_COUNTS):
        raise SystemExit(f"Planner family counts changed: {dict(counts)}")

    for planner in planners:
        planner_id = planner["id"]
        lesson = lessons[planner_id]
        if not all(
            lesson.get(field)
            for field in ("tagline", "story", "experiment", "question", "answer")
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

    return planners, rows, project_map


def source_url(path: str) -> str:
    target = PROJECT_ROOT / path
    endpoint = "tree" if target.is_dir() else "blob"
    return f"https://github.com/damminhtien/pathplanning/{endpoint}/main/{path}"


def render_static_catalog(planners: list[dict[str, Any]]) -> str:
    cards = []
    for planner in planners:
        planner_id = html.escape(planner["id"], quote=True)
        title = html.escape(planner["title"])
        description = html.escape(planner["description"])
        kind = html.escape(FAMILY_LABELS[planner["kind"]])
        principle = html.escape(planner["principle"])
        cards.append(
            f'<article class="planner-card"><div class="planner-card-top">'
            f'<span class="planner-family">{kind}</span></div>'
            f'<h3><a class="planner-card-title" href="algorithms/{planner_id}/">{title}</a></h3>'
            f'<p>{description}</p><div class="planner-card-footer">'
            f'<span class="planner-principle">{principle}</span>'
            f'<a class="card-lesson" href="algorithms/{planner_id}/">Read lesson ↗</a>'
            f'</div></article>'
        )
    return '\n'.join(cards)


def json_ld(value: dict[str, Any]) -> str:
    return json.dumps(value, ensure_ascii=False).replace("</", "<\\/")


def lesson_page(
    planner: dict[str, Any],
    lesson: dict[str, Any],
    registry_row: dict[str, str],
    planner_by_id: dict[str, dict[str, Any]],
    planner_positions: dict[str, int],
    output_root: Path,
    output_file: Path,
) -> str:
    planner_id = planner["id"]
    title = planner["title"]
    kind = planner["kind"]
    page_url = f"{BASE_URL}/algorithms/{planner_id}/"
    description = (
        f"{lesson['tagline']} Learn the idea, trade-off, and motion-planning concept."
    )[:158].rsplit(" ", 1)[0]
    depth = len(output_file.parent.relative_to(output_root).parts)
    asset_root = "../" * depth
    github_module = registry_row["module"].replace(".", "/") + ".py"
    module_url = source_url(github_module)
    related: list[str] = []
    for related_id in lesson["related"]:
        related_planner = planner_by_id[related_id]
        related.append(
            f'<a class="related-card" href="../{html.escape(related_id)}/">'
            f'<span>{html.escape(FAMILY_LABELS[related_planner["kind"]])}</span>'
            f'<strong>{html.escape(related_planner["title"])}</strong>'
            f'<small>{html.escape(related_planner["principle"])}</small></a>'
        )

    steps = "\n".join(
        f'<li><span class="lesson-step-number">0{index}</span>'
        f'<p>{html.escape(step)}</p></li>'
        for index, step in enumerate(lesson["steps"], start=1)
    )
    example_steps = "\n".join(
        f'<li><span class="lesson-step-number">0{index}</span>'
        f'<p>{html.escape(step)}</p></li>'
        for index, step in enumerate(lesson["example"]["steps"], start=1)
    )
    metric_tags = "\n".join(
        f'<li>{html.escape(metric)}</li>' for metric in lesson["metrics"]
    )
    schema = json_ld(
        {
            "@context": "https://schema.org",
            "@type": "LearningResource",
            "name": f"{title} Explained",
            "headline": title,
            "description": description,
            "url": page_url,
            "inLanguage": "en",
            "learningResourceType": "Interactive tutorial",
            "isAccessibleForFree": True,
            "about": [
                {"@type": "Thing", "name": title},
                {"@type": "Thing", "name": "Robot motion planning"},
                {"@type": "Thing", "name": "Autonomous agents"},
            ],
            "teaches": [
                "Robot motion planning",
                "Autonomous agent path planning",
                planner["principle"],
            ],
            "educationalUse": "Instruction",
            "audience": {"@type": "EducationalAudience", "educationalRole": "student"},
            "keywords": [title, "robot motion planning", "path planning algorithm", "autonomous agents"],
            "isPartOf": {
                "@type": "WebSite",
                "name": "Pathplanning Learning Lab",
                "url": f"{BASE_URL}/",
            },
        }
    )
    breadcrumb_schema = json_ld(
        {
            "@context": "https://schema.org",
            "@type": "BreadcrumbList",
            "itemListElement": [
                {"@type": "ListItem", "position": 1, "name": "Home", "item": f"{BASE_URL}/"},
                {"@type": "ListItem", "position": 2, "name": "Algorithm atlas", "item": f"{BASE_URL}/#algorithms"},
                {"@type": "ListItem", "position": 3, "name": title, "item": page_url},
            ],
        }
    )
    return f"""<!doctype html>
<html lang="en">
  <head>
    <meta charset="utf-8" />
    <meta name="viewport" content="width=device-width, initial-scale=1" />
    <meta name="theme-color" content="#f8fbff" />
    <meta name="robots" content="index, follow, max-image-preview:large" />
    <meta name="description" content="{html.escape(description, quote=True)}" />
    <link rel="canonical" href="{page_url}" />
    <link rel="icon" href="{asset_root}favicon.svg" type="image/svg+xml" />
    <link rel="stylesheet" href="{asset_root}styles.css" />
    <link rel="stylesheet" href="{asset_root}theme.css" />
    <link rel="stylesheet" href="{asset_root}learning.css" />
    <meta property="og:type" content="article" />
    <meta property="article:section" content="{html.escape(FAMILY_LABELS[kind], quote=True)}" />
    <meta property="og:site_name" content="Pathplanning Learning Lab" />
    <meta property="og:title" content="{html.escape(title)} Explained | Pathplanning" />
    <meta property="og:description" content="{html.escape(description, quote=True)}" />
    <meta property="og:url" content="{page_url}" />
    <meta property="og:image" content="{BASE_URL}/og-image.png" />
    <meta property="og:image:alt" content="A robot finding a route around obstacles" />
    <meta name="twitter:card" content="summary_large_image" />
    <meta name="twitter:title" content="{html.escape(title)} Explained | Pathplanning" />
    <meta name="twitter:description" content="{html.escape(description, quote=True)}" />
    <meta name="twitter:image" content="{BASE_URL}/og-image.png" />
    <meta name="twitter:image:alt" content="A robot finding a route around obstacles" />
    <script type="application/ld+json">{schema}</script>
    <script type="application/ld+json">{breadcrumb_schema}</script>
    <title>{html.escape(title)} Explained | Pathplanning Tutorial</title>
  </head>
  <body class="learning-body lesson-body family-{html.escape(kind)}">
    <a class="skip-link" href="#lesson-main">Skip to lesson</a>
    <header class="learning-topbar">
      <a class="brand" href="{asset_root}" aria-label="Pathplanning Learning Lab home">
        <span class="brand-mark" aria-hidden="true">↗</span>
        <span>path<span class="brand-accent">planning</span></span>
      </a>
      <nav aria-label="Learning navigation">
        <a href="{asset_root}#algorithms">Algorithm atlas</a>
        <a href="{asset_root}mindmap/">Project mind map</a>
        <a href="https://github.com/damminhtien/pathplanning" target="_blank" rel="noreferrer">Source ↗</a>
      </nav>
    </header>
    <main id="lesson-main" class="lesson-shell">
      <nav class="breadcrumbs" aria-label="Breadcrumb">
        <a href="{asset_root}">Home</a><span aria-hidden="true">/</span>
        <a href="{asset_root}#algorithms">Algorithm atlas</a><span aria-hidden="true">/</span>
        <span aria-current="page">{html.escape(title)}</span>
      </nav>
      <header class="lesson-hero">
        <div class="lesson-kicker"><span class="family-dot"></span>{html.escape(FAMILY_LABELS[kind])} · LESSON {planner_positions[planner_id]:02d} / 37</div>
        <h1>{html.escape(title)}</h1>
        <p class="lesson-tagline">{html.escape(lesson["tagline"])}</p>
        <p class="lesson-summary">{html.escape(planner["description"])}</p>
        <a class="lesson-principle" href="#mechanism"><span>The little rule</span><code>{html.escape(planner["principle"])}</code></a>
      </header>

      <div class="lesson-layout">
        <aside class="lesson-rail" aria-label="On this page">
          <p>ON THIS PAGE</p>
          <a href="#big-picture">The big picture</a>
          <a href="#watch">Watch it think</a>
          <a href="#mechanism">How it decides</a>
          <a href="#example">Worked example</a>
          <a href="#tradeoff">Fit & trade-off</a>
          <a href="#evaluation">Evaluate fairly</a>
          <a href="#try-it">Your turn</a>
        </aside>
        <article class="lesson-content">
          <section id="big-picture" class="lesson-section">
            <p class="lesson-section-label">01 · THE BIG PICTURE</p>
            <h2>A route is a chain of choices.</h2>
            <p>{html.escape(lesson["story"])}</p>
          </section>

          <section id="watch" class="lesson-section lesson-watch">
            <div class="lesson-section-label">02 · WATCH IT THINK</div>
            <h2>Follow the idea, one frame at a time.</h2>
            <p>Run the illustration, pause on a decision, or reset it and look again. The colors tell a small story about this planner's search.</p>
            <div class="lesson-demo-frame">
              <iframe
                title="Interactive concept animation for {html.escape(title, quote=True)}"
                src="{asset_root}?demo={html.escape(planner_id)}"
                loading="lazy"
                referrerpolicy="strict-origin-when-cross-origin"></iframe>
            </div>
            <p class="lesson-caption">A browser-based concept animation, not an execution of the Python/C++ planner or a benchmark result.</p>
          </section>

          <section id="mechanism" class="lesson-section">
            <p class="lesson-section-label">03 · THE DECISION RULE</p>
            <h2>How does it decide what to do next?</h2>
            <p>{html.escape(lesson["mechanism"])}</p>
            <h3 class="lesson-subheading">The search, in three moves</h3>
            <ol class="lesson-steps">{steps}</ol>
          </section>

          <section id="example" class="lesson-section lesson-example">
            <p class="lesson-section-label">04 · WORK THROUGH AN EXAMPLE</p>
            <h2>Work through a small case.</h2>
            <p class="lesson-example-setup">{html.escape(lesson["example"]["setup"])}</p>
            <ol class="lesson-steps lesson-example-steps">{example_steps}</ol>
          </section>

          <section id="tradeoff" class="lesson-section lesson-tradeoff">
            <p class="lesson-section-label">05 · WHEN IT FITS</p>
            <h2>Choose it for the problem it solves.</h2>
            <div class="lesson-fit-grid">
              <article class="lesson-fit-card">
                <h3>A good fit</h3>
                <p>{html.escape(lesson["best_for"])}</p>
              </article>
              <article class="lesson-fit-card lesson-fit-caution">
                <h3>Watch out</h3>
                <p>{html.escape(lesson["watch_out"])}</p>
              </article>
            </div>
            <p class="lesson-project-tradeoff">In this project: {html.escape(planner["tradeoff"])}</p>
            <div class="contract-note"><span aria-hidden="true">↗</span><p>Algorithm guarantees depend on the graph, goal, costs, parameters, and stop reason. The project documents those exact conditions in its supported planner matrix.</p></div>
            <a href="https://github.com/damminhtien/pathplanning/blob/main/SUPPORTED_ALGORITHMS.md" target="_blank" rel="noreferrer">Read the supported contract ↗</a>
          </section>

          <section id="evaluation" class="lesson-section lesson-evaluation">
            <p class="lesson-section-label">06 · EVALUATE IT FAIRLY</p>
            <h2>What would make this a fair comparison?</h2>
            <p>{html.escape(lesson["evaluation"])}</p>
            <h3 class="lesson-subheading">Useful measures for this planner</h3>
            <ul class="lesson-metrics">{metric_tags}</ul>
            <a class="lesson-method-link" href="{asset_root}#method">Explore this project's evaluation method ↗</a>
          </section>

          <section id="try-it" class="lesson-section lesson-try">
            <p class="lesson-section-label">07 · YOUR TURN</p>
            <h2>{html.escape(lesson["experiment"])}</h2>
            <details class="lesson-question">
              <summary>{html.escape(lesson["question"])}</summary>
              <p>{html.escape(lesson["answer"])}</p>
            </details>
          </section>

          <section class="lesson-source">
            <div><p class="lesson-section-label">IN THIS PROJECT</p><h2>One idea, one callable.</h2></div>
            <dl>
              <div><dt>Planner ID</dt><dd><code>{html.escape(planner_id)}</code></dd></div>
              <div><dt>Module</dt><dd><code>{html.escape(github_module)}</code></dd></div>
              <div><dt>Callable</dt><dd><code>{html.escape(registry_row["callable"])}</code></dd></div>
            </dl>
            <a class="button button-primary" href="{module_url}" target="_blank" rel="noreferrer">Read the implementation ↗</a>
          </section>

          <section class="related-lessons">
            <p class="lesson-section-label">FOLLOW THE THREAD</p>
            <h2>Two nearby ideas</h2>
            <div class="related-grid">{''.join(related)}</div>
          </section>
        </article>
      </div>
    </main>
    <footer class="learning-footer">
      <a href="{asset_root}">← Back to the learning lab</a>
      <a href="{asset_root}mindmap/">Explore the whole project map →</a>
    </footer>
  </body>
</html>
"""


def expand_project_map(
    node: dict[str, Any],
    planners: list[dict[str, Any]],
    registry_rows: dict[str, dict[str, str]],
) -> dict[str, Any]:
    expanded = dict(node)
    if node.get("childrenFrom") == "plannerKinds":
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
            expand_project_map(child, planners, registry_rows)
            for child in node["children"]
        ]
    return expanded


def render_map_nodes(nodes: list[dict[str, Any]], depth: int = 0) -> str:
    items: list[str] = []
    for node in nodes:
        label = html.escape(node["label"])
        summary = html.escape(node["summary"])
        search_text = html.escape(
            f"{node['label']} {node['summary']} {node.get('path', '')}".lower(), quote=True
        )
        path = node.get("path", "")
        page = node.get("page", "")
        node_type = html.escape(node.get("type", "concept"))
        details = ""
        if path:
            source = html.escape(source_url(path), quote=True)
            path_text = html.escape(path)
            details += (
                f'<a class="mindmap-source" href="{source}" target="_blank" rel="noreferrer">'
                f'<code>{path_text}</code><span>Open source ↗</span></a>'
            )
        if page:
            lesson_href = html.escape(f"{BASE_URL}{page}", quote=True)
            principle = html.escape(node.get("principle", ""))
            details += (
                f'<a class="mindmap-lesson" href="{lesson_href}">'
                f'<span>Read the lesson</span><code>{principle}</code></a>'
            )
        children = render_map_nodes(node.get("children", []), depth + 1)
        children_markup = (
            f'<ul class="mindmap-children">{children}</ul>' if children else ""
        )
        open_attr = " open" if depth == 0 else ""
        items.append(
            f'<li class="mindmap-item"><details class="mindmap-node" '
            f'data-id="{html.escape(node["id"], quote=True)}" data-type="{node_type}" '
            f'data-search="{search_text}"{open_attr}>'
            f'<summary><span class="mindmap-dot" aria-hidden="true"></span>'
            f'<span class="mindmap-label">{label}</span>'
            f'<span class="mindmap-type">{node_type.replace("-", " ")}</span></summary>'
            f'<div class="mindmap-content"><p>{summary}</p>{details}'
            f'<button class="mindmap-focus" type="button">Focus this idea</button>'
            f'{children_markup}</div></details></li>'
        )
    return "\n".join(items)


def map_page(map_data: dict[str, Any]) -> str:
    tree = render_map_nodes(map_data["children"])
    page_url = f"{BASE_URL}/mindmap/"
    description = (
        "Explore an interactive mind map of the pathplanning toolkit: worlds, "
        "37 planners, APIs, native engines, geometry, data, and evaluation."
    )
    schema = json_ld(
        {
            "@context": "https://schema.org",
            "@type": "LearningResource",
            "name": "Pathplanning Project Mind Map",
            "description": description,
            "url": page_url,
            "inLanguage": "en",
            "learningResourceType": "Interactive project map",
            "isAccessibleForFree": True,
            "about": [
                {"@type": "Thing", "name": "Robot motion planning"},
                {"@type": "Thing", "name": "Path planning algorithms"},
                {"@type": "Thing", "name": "Autonomous agents"},
            ],
        }
    )
    breadcrumb_schema = json_ld(
        {
            "@context": "https://schema.org",
            "@type": "BreadcrumbList",
            "itemListElement": [
                {"@type": "ListItem", "position": 1, "name": "Home", "item": f"{BASE_URL}/"},
                {"@type": "ListItem", "position": 2, "name": "Project mind map", "item": page_url},
            ],
        }
    )
    return f"""<!doctype html>
<html lang="en">
  <head>
    <meta charset="utf-8" />
    <meta name="viewport" content="width=device-width, initial-scale=1" />
    <meta name="theme-color" content="#f8fbff" />
    <meta name="robots" content="index, follow, max-image-preview:large" />
    <meta name="description" content="{html.escape(description, quote=True)}" />
    <link rel="canonical" href="{page_url}" />
    <link rel="icon" href="../favicon.svg" type="image/svg+xml" />
    <link rel="stylesheet" href="../styles.css" />
    <link rel="stylesheet" href="../theme.css" />
    <link rel="stylesheet" href="../learning.css" />
    <link rel="stylesheet" href="../mindmap.css" />
    <meta property="og:type" content="website" />
    <meta property="og:site_name" content="Pathplanning Learning Lab" />
    <meta property="og:title" content="The Pathplanning Project Mind Map" />
    <meta property="og:description" content="{html.escape(description, quote=True)}" />
    <meta property="og:url" content="{page_url}" />
    <meta property="og:image" content="{BASE_URL}/og-image.png" />
    <meta name="twitter:card" content="summary_large_image" />
    <meta name="twitter:title" content="The Pathplanning Project Mind Map" />
    <meta name="twitter:description" content="{html.escape(description, quote=True)}" />
    <meta name="twitter:image" content="{BASE_URL}/og-image.png" />
    <script type="application/ld+json">{schema}</script>
    <script type="application/ld+json">{breadcrumb_schema}</script>
    <script src="../mindmap.js" defer></script>
    <title>The Pathplanning Project Mind Map | 37 Algorithms & Toolkit</title>
  </head>
  <body class="learning-body mindmap-body">
    <a class="skip-link" href="#project-map">Skip to project map</a>
    <header class="learning-topbar">
      <a class="brand" href="../" aria-label="Pathplanning Learning Lab home">
        <span class="brand-mark" aria-hidden="true">↗</span>
        <span>path<span class="brand-accent">planning</span></span>
      </a>
      <nav aria-label="Learning navigation">
        <a href="../#algorithms">Algorithm atlas</a>
        <a href="../#evaluation">Evaluation</a>
        <a href="https://github.com/damminhtien/pathplanning" target="_blank" rel="noreferrer">Source ↗</a>
      </nav>
    </header>
    <main class="map-shell">
      <header class="map-hero">
        <p class="lesson-section-label">A MAP OF THE WHOLE TOOLKIT</p>
        <h1>One project.<br /><span>Many ways through.</span></h1>
        <p>{html.escape(map_data["summary"])}</p>
      </header>
      <section class="map-controls" aria-label="Mind map controls">
        <label class="map-search"><span aria-hidden="true">⌕</span><span class="sr-only">Search the project map</span>
          <input id="map-search" type="search" placeholder="Find an algorithm, module, or idea…" autocomplete="off" />
        </label>
        <div class="map-actions">
          <button type="button" id="expand-map">Open all branches</button>
          <button type="button" id="collapse-map">Close branches</button>
        </div>
      </section>
      <div class="map-layout">
        <section class="mindmap-stage" id="project-map" aria-label="Interactive pathplanning project map">
          <div class="map-root"><span class="map-root-orbit" aria-hidden="true"></span><strong>pathplanning</strong><small>observe · model · plan · check · move</small></div>
          <ul class="mindmap-branches">{tree}</ul>
          <p class="map-empty" id="map-empty" hidden>No matching ideas. Try a planner name, family, file, or concept.</p>
        </section>
        <aside class="map-inspector" id="map-inspector" aria-live="polite" aria-atomic="true">
          <p class="lesson-section-label">FOLLOW A BRANCH</p>
          <h2 id="inspector-title">{html.escape(map_data["label"])}</h2>
          <p id="inspector-summary">{html.escape(map_data["summary"])}</p>
          <a id="inspector-source" class="mindmap-source" href="https://github.com/damminhtien/pathplanning" target="_blank" rel="noreferrer" hidden>
            <code id="inspector-path"></code><span>Open source ↗</span>
          </a>
          <a id="inspector-lesson" class="button button-primary" href="../" hidden>Read the lesson ↗</a>
          <p class="inspector-hint">Open a branch to see what it contains. Select any node to focus this panel.</p>
        </aside>
      </div>
      <section class="map-reading">
        <p class="lesson-section-label">HOW TO READ THIS MAP</p>
        <p>The branches connect the agent's problem, the planner family, the API and native engine, and the evidence used to check a route. A planner only makes sense inside those connections.</p>
        <a href="../">Return to the interactive learning lab →</a>
      </section>
    </main>
  </body>
</html>
"""


def build_sitemap(planners: list[dict[str, Any]], today: str) -> str:
    namespace = "http://www.sitemaps.org/schemas/sitemap/0.9"
    ET.register_namespace("", namespace)
    root = ET.Element(f"{{{namespace}}}urlset")
    urls = [f"{BASE_URL}/", f"{BASE_URL}/mindmap/"]
    urls.extend(f"{BASE_URL}/algorithms/{planner['id']}/" for planner in planners)
    for url in urls:
        entry = ET.SubElement(root, f"{{{namespace}}}url")
        ET.SubElement(entry, f"{{{namespace}}}loc").text = url
        ET.SubElement(entry, f"{{{namespace}}}lastmod").text = today
    ET.indent(root, space="  ")
    return ET.tostring(root, encoding="unicode", xml_declaration=True) + "\n"


def prepare_output(output: Path) -> Path:
    output = output.resolve()
    if output in (WEBSITE_ROOT, PROJECT_ROOT, Path("/")):
        raise SystemExit(f"Refusing to use a source directory as output: {output}")
    if output.exists():
        if output.is_symlink() or not (output / BUILD_MARKER).is_file():
            raise SystemExit(f"Output exists and is not owned by this builder: {output}")
        shutil.rmtree(output)
    output.mkdir(parents=True)
    (output / BUILD_MARKER).write_text("Generated by website/build_site.py\n", encoding="utf-8")
    return output


def copy_static_files(output: Path) -> None:
    for name in (
        "index.html",
        "app.js",
        "styles.css",
        "theme.css",
        "favicon.svg",
        "og-image.png",
        "og-image.svg",
        "404.html",
        "robots.txt",
    ):
        shutil.copy2(WEBSITE_ROOT / name, output / name)
    shutil.copytree(WEBSITE_ROOT / "data", output / "data")
    for name in ("learning.css", "mindmap.css", "mindmap.js"):
        shutil.copy2(WEBSITE_ROOT / name, output / name)


def build(output: Path) -> int:
    planners, rows, map_data = load_catalog()
    lessons = read_json(WEBSITE_ROOT / "data" / "tutorials.json")
    if len(planners) != 37:
        raise SystemExit(f"Expected 37 supported planner tutorials; found {len(planners)}.")
    expanded_map = expand_project_map(map_data, planners, rows)

    output = prepare_output(output)
    copy_static_files(output)
    home = output / "index.html"
    home_source = home.read_text(encoding="utf-8")
    empty_catalog = '<div class="planner-grid" id="planner-grid"></div>'
    static_catalog = '<div class="planner-grid" id="planner-grid">' + render_static_catalog(planners) + '</div>'
    if empty_catalog not in home_source:
        raise SystemExit("Could not find the planner catalog placeholder on the home page.")
    home.write_text(home_source.replace(empty_catalog, static_catalog, 1), encoding="utf-8")
    today = dt.datetime.now(dt.timezone.utc).date().isoformat()
    sitemap = build_sitemap(planners, today)
    (output / "sitemap.xml").write_text(sitemap, encoding="utf-8")
    (WEBSITE_ROOT / "sitemap.xml").write_text(sitemap, encoding="utf-8")

    planner_by_id = {planner["id"]: planner for planner in planners}
    planner_positions = {planner["id"]: index for index, planner in enumerate(planners, start=1)}
    for planner in planners:
        page_dir = output / "algorithms" / planner["id"]
        page_dir.mkdir(parents=True)
        page_file = page_dir / "index.html"
        page_file.write_text(
            lesson_page(
                planner,
                lessons[planner["id"]],
                rows[planner["id"]],
                planner_by_id,
                planner_positions,
                output,
                page_file,
            ),
            encoding="utf-8",
        )

    map_dir = output / "mindmap"
    map_dir.mkdir()
    (map_dir / "index.html").write_text(map_page(expanded_map), encoding="utf-8")
    print(f"Built the home page, project mind map, and {len(planners)} planner lessons in {output}")
    return len(planners)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=WEBSITE_ROOT / "dist")
    args = parser.parse_args()
    build(args.output)
    return 0


if __name__ == "__main__":
    sys.exit(main())
