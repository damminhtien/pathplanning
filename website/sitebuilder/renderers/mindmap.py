"""Renderer for the searchable project architecture map."""

from __future__ import annotations

import html
from typing import Any

from ..config import BASE_URL
from ..content import source_url
from ..math_text import render_math_text
from .catalog import json_ld


def render_map_nodes(nodes: list[dict[str, Any]], depth: int = 0) -> str:
    items: list[str] = []
    for node in nodes:
        label = html.escape(node["label"])
        summary = render_math_text(node["summary"])
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
                f"<code>{path_text}</code><span>Open source</span></a>"
            )
        if page:
            lesson_href = html.escape(f"{BASE_URL}{page}", quote=True)
            principle = render_math_text(node.get("principle", ""))
            page_label = html.escape(node.get("page_label", "Read the lesson"))
            principle_markup = (
                f'<span class="mindmap-principle">{principle}</span>' if principle else ""
            )
            details += (
                f'<a class="mindmap-lesson" href="{lesson_href}">'
                f"<span>{page_label}</span>{principle_markup}</a>"
            )
        children = render_map_nodes(node.get("children", []), depth + 1)
        children_markup = f'<ul class="mindmap-children">{children}</ul>' if children else ""
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
            f"{children_markup}</div></details></li>"
        )
    return "\n".join(items)


def map_page(map_data: dict[str, Any], planner_count: int) -> str:
    tree = render_map_nodes(map_data["children"])
    page_url = f"{BASE_URL}/mindmap/"
    description = (
        "Explore an interactive mind map of the pathplanning toolkit: worlds, "
        f"{planner_count} planners, APIs, native engines, geometry, data, and evaluation."
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
    <script src="../mathjax-config.js"></script>
    <script async id="mathjax-script" src="https://cdn.jsdelivr.net/npm/mathjax@4/tex-chtml.js"></script>
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
    <title>The Pathplanning Project Mind Map | {planner_count} Algorithms &amp; Toolkit</title>
  </head>
  <body class="learning-body mindmap-body">
    <a class="skip-link" href="#project-map">Skip to project map</a>
    <header class="learning-topbar">
      <a class="brand" href="../" aria-label="Pathplanning Learning Lab home">
        <span>path<span class="brand-accent">planning</span></span>
      </a>
      <nav aria-label="Learning navigation">
        <a href="../#algorithms">Algorithm atlas</a>
        <a href="../datasets/">Datasets</a>
        <a href="../case-studies/">Case studies</a>
        <a class="nav-evaluation" href="../#evaluation">Evaluation</a>
        <a class="nav-source" href="https://github.com/damminhtien/pathplanning" target="_blank" rel="noreferrer">Source</a>
      </nav>
    </header>
    <main class="map-shell">
      <header class="map-hero">
        <p class="lesson-section-label">A MAP OF THE WHOLE TOOLKIT</p>
        <h1>One project.<br /><span>Many ways through.</span></h1>
        <p>{html.escape(map_data["summary"])}</p>
      </header>
      <section class="map-controls" aria-label="Mind map controls">
        <label class="map-search"><span class="sr-only">Search the project map</span>
          <input id="map-search" type="search" placeholder="Find an algorithm, module, or idea" autocomplete="off" />
        </label>
        <div class="map-actions">
          <button type="button" id="expand-map">Open all branches</button>
          <button type="button" id="collapse-map">Close branches</button>
        </div>
      </section>
      <div class="map-layout">
        <section class="mindmap-stage" id="project-map" aria-label="Interactive pathplanning project map">
          <div class="map-root"><span class="map-root-orbit" aria-hidden="true"></span><strong>pathplanning</strong><small>observe, model, plan, check, move</small></div>
          <ul class="mindmap-branches">{tree}</ul>
          <p class="map-empty" id="map-empty" hidden>No matching ideas. Try a planner name, family, file, or concept.</p>
        </section>
        <aside class="map-inspector" id="map-inspector" aria-live="polite" aria-atomic="true">
          <p class="lesson-section-label">FOLLOW A BRANCH</p>
          <h2 id="inspector-title">{html.escape(map_data["label"])}</h2>
          <p id="inspector-summary">{html.escape(map_data["summary"])}</p>
          <a id="inspector-source" class="mindmap-source" href="https://github.com/damminhtien/pathplanning" target="_blank" rel="noreferrer" hidden>
            <code id="inspector-path"></code><span>Open source</span>
          </a>
          <a id="inspector-lesson" class="button button-primary" href="../" hidden>Read the lesson</a>
          <p class="inspector-hint">Open a branch to see what it contains. Select any node to focus this panel.</p>
        </aside>
      </div>
      <section class="map-reading">
        <p class="lesson-section-label">HOW TO READ THIS MAP</p>
        <p>The branches connect the agent's problem, the planner family, the API and native engine, and the evidence used to check a route. A planner only makes sense inside those connections.</p>
        <a href="../">Return to the interactive learning lab</a>
      </section>
    </main>
  </body>
</html>
"""
