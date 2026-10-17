"""Render the dataset catalog and its source-specific learning pages."""

from __future__ import annotations

import html
from pathlib import Path
from typing import Any

from ..config import BASE_URL
from ..content import source_url
from .catalog import json_ld


def _e(value: str) -> str:
    return html.escape(value, quote=True)


def _relative_root(output_root: Path, output_file: Path) -> str:
    depth = len(output_file.parent.relative_to(output_root).parts)
    return "../" * depth


def _page_head(
    *,
    title: str,
    description: str,
    page_url: str,
    asset_root: str,
    social_type: str,
    schema: str,
    breadcrumb_schema: str,
    extra_stylesheets: tuple[str, ...] = ("datasets.css",),
) -> str:
    stylesheet_links = "\n".join(
        f'    <link rel="stylesheet" href="{asset_root}{_e(name)}" />'
        for name in extra_stylesheets
    )
    return f"""<!doctype html>
<html lang="en">
  <head>
    <meta charset="utf-8" />
    <meta name="viewport" content="width=device-width, initial-scale=1" />
    <meta name="theme-color" content="#f8fbff" />
    <meta name="robots" content="index, follow, max-image-preview:large" />
    <meta name="description" content="{_e(description)}" />
    <link rel="canonical" href="{_e(page_url)}" />
    <link rel="icon" href="{asset_root}favicon.svg" type="image/svg+xml" />
    <link rel="stylesheet" href="{asset_root}styles.css" />
    <link rel="stylesheet" href="{asset_root}theme.css" />
    <link rel="stylesheet" href="{asset_root}learning.css" />
{stylesheet_links}
    <meta property="og:type" content="{_e(social_type)}" />
    <meta property="og:site_name" content="Pathplanning Learning Lab" />
    <meta property="og:title" content="{_e(title)}" />
    <meta property="og:description" content="{_e(description)}" />
    <meta property="og:url" content="{_e(page_url)}" />
    <meta property="og:image" content="{BASE_URL}/og-image.png" />
    <meta property="og:image:alt" content="Pathplanning Learning Lab benchmark and dataset guide" />
    <meta name="twitter:card" content="summary_large_image" />
    <meta name="twitter:title" content="{_e(title)}" />
    <meta name="twitter:description" content="{_e(description)}" />
    <meta name="twitter:image" content="{BASE_URL}/og-image.png" />
    <meta name="twitter:image:alt" content="Pathplanning Learning Lab benchmark and dataset guide" />
    <script type="application/ld+json">{schema}</script>
    <script type="application/ld+json">{breadcrumb_schema}</script>
    <title>{_e(title)}</title>
  </head>"""


def _learning_nav(asset_root: str) -> str:
    return f"""<header class="learning-topbar">
      <a class="brand" href="{asset_root}" aria-label="Pathplanning Learning Lab home">
        <span>path<span class="brand-accent">planning</span></span>
      </a>
      <nav aria-label="Learning navigation">
        <a href="{asset_root}#algorithms">Algorithm atlas</a>
        <a href="{asset_root}datasets/">Datasets</a>
        <a href="{asset_root}case-studies/">Case studies</a>
        <a class="nav-evaluation" href="{asset_root}#evaluation">Evaluation</a>
        <a class="nav-project-map" href="{asset_root}mindmap/">Project map</a>
        <a class="nav-source" href="https://github.com/damminhtien/pathplanning" target="_blank" rel="noreferrer">Source</a>
      </nav>
    </header>"""


def _breadcrumb(items: list[tuple[str, str | None]]) -> str:
    crumb_items = []
    for index, (label, url) in enumerate(items):
        if index:
            crumb_items.append('<span aria-hidden="true">/</span>')
        if url:
            crumb_items.append(f'<a href="{_e(url)}">{_e(label)}</a>')
        else:
            crumb_items.append(f'<span aria-current="page">{_e(label)}</span>')
    return '<nav class="breadcrumbs" aria-label="Breadcrumb">' + "".join(crumb_items) + "</nav>"


def _stat_cards(stats: list[dict[str, str]]) -> str:
    return "\n".join(
        f'<div class="dataset-stat"><strong>{_e(row["value"])}</strong>'
        f"<span>{_e(row['label'])}</span></div>"
        for row in stats
    )


def _render_table(table: dict[str, Any]) -> str:
    headers = "".join(f'<th scope="col">{_e(cell)}</th>' for cell in table["headers"])
    rows = []
    for row in table["rows"]:
        cells = "".join(f"<td>{_e(cell)}</td>" for cell in row)
        rows.append(f"<tr>{cells}</tr>")
    caption = f"<caption>{_e(table['caption'])}</caption>" if table.get("caption") else ""
    return (
        '<div class="dataset-table-wrap"><table class="dataset-table">'
        f"{caption}<thead><tr>{headers}</tr></thead><tbody>{''.join(rows)}</tbody></table></div>"
    )


def _section(section: dict[str, Any]) -> str:
    paragraphs = "".join(f"<p>{_e(paragraph)}</p>" for paragraph in section.get("paragraphs", []))
    bullets = section.get("bullets", [])
    list_markup = ""
    if bullets:
        list_markup = "<ul>" + "".join(f"<li>{_e(item)}</li>" for item in bullets) + "</ul>"
    table_markup = _render_table(section["table"]) if section.get("table") else ""
    callout = section.get("callout")
    callout_markup = ""
    if callout:
        callout_markup = (
            '<aside class="dataset-callout"><strong>'
            f"{_e(callout['title'])}</strong><p>{_e(callout['text'])}</p></aside>"
        )
    return f"""<section class="dataset-section" id="{_e(section["id"])}">
      <p class="lesson-section-label">{_e(section["label"])}</p>
      <h2>{_e(section["title"])}</h2>
      {paragraphs}{list_markup}{table_markup}{callout_markup}
    </section>"""


def _source_cards(sources: list[dict[str, str]]) -> str:
    return "\n".join(
        f'<a class="dataset-source-card" href="{_e(source["url"])}" '
        f'target="_blank" rel="noreferrer"><strong>{_e(source["name"])}</strong>'
        f"<span>{_e(source['context'])}</span><small>Open source ↗</small></a>"
        for source in sources
    )


def _project_links(paths: list[str]) -> str:
    return "\n".join(
        f'<a class="dataset-file-link" href="{_e(source_url(path))}" '
        f'target="_blank" rel="noreferrer"><code>{_e(path)}</code><span>View on GitHub</span></a>'
        for path in paths
    )


def dataset_index_page(
    datasets: list[dict[str, Any]],
    snapshot: dict[str, Any],
    output_root: Path,
    output_file: Path,
) -> str:
    page_url = f"{BASE_URL}/datasets/"
    title = "Motion Planning Benchmark Datasets: Maps, Graphs & Robot Worlds | Pathplanning"
    description = (
        "Explore detailed guides to the MovingAI, DIMACS, BARN, Monash, and OMPL benchmark "
        "data used by Pathplanning. See data formats, movement rules, evaluation coverage, "
        "and limitations for each collection."
    )
    asset_root = _relative_root(output_root, output_file)
    cards = []
    for dataset in datasets:
        stats = "\n".join(
            f"<div><strong>{_e(row['value'])}</strong><span>{_e(row['label'])}</span></div>"
            for row in dataset["stats"][:2]
        )
        cards.append(
            f'<article class="dataset-profile-card">'
            f'<p class="dataset-card-kicker">{_e(dataset["eyebrow"])}</p>'
            f'<h2><a href="{_e(dataset["id"])}/">{_e(dataset["title"])}</a></h2>'
            f'<p class="dataset-card-description">{_e(dataset["tagline"])}</p>'
            f'<div class="dataset-card-stats">{stats}</div>'
            f'<div class="dataset-card-status"><b>Campaign status</b>'
            f"<p>{_e(dataset['campaign_status'])}</p></div>"
            f'<a class="dataset-card-link" href="{_e(dataset["id"])}/">Read the full dataset profile <span aria-hidden="true">→</span></a>'
            f"</article>"
        )

    schema = json_ld(
        {
            "@context": "https://schema.org",
            "@type": "CollectionPage",
            "name": "Motion Planning Benchmark Datasets",
            "description": description,
            "url": page_url,
            "inLanguage": "en",
            "isPartOf": {
                "@type": "WebSite",
                "name": "Pathplanning Learning Lab",
                "url": f"{BASE_URL}/",
            },
            "mainEntity": {
                "@type": "ItemList",
                "numberOfItems": len(datasets),
                "itemListElement": [
                    {
                        "@type": "ListItem",
                        "position": index,
                        "name": dataset["title"],
                        "url": f"{BASE_URL}/datasets/{dataset['id']}/",
                    }
                    for index, dataset in enumerate(datasets, start=1)
                ],
            },
        }
    )
    breadcrumb_schema = json_ld(
        {
            "@context": "https://schema.org",
            "@type": "BreadcrumbList",
            "itemListElement": [
                {"@type": "ListItem", "position": 1, "name": "Home", "item": f"{BASE_URL}/"},
                {"@type": "ListItem", "position": 2, "name": "Dataset guide", "item": page_url},
            ],
        }
    )
    head = _page_head(
        title=title,
        description=description,
        page_url=page_url,
        asset_root=asset_root,
        social_type="website",
        schema=schema,
        breadcrumb_schema=breadcrumb_schema,
    )
    return f"""{head}
  <body class="learning-body dataset-body dataset-index-body">
    <a class="skip-link" href="#dataset-main">Skip to dataset guide</a>
    {_learning_nav(asset_root)}
    <main id="dataset-main" class="datasets-shell">
      {_breadcrumb([("Home", asset_root), ("Datasets", None)])}
      <header class="dataset-index-hero">
        <p class="lesson-kicker">BENCHMARK DATA / A FIELD GUIDE</p>
        <h1>The worlds behind<br /><span>the search.</span></h1>
        <p class="dataset-lede">A planner only answers a question about the world it was given. These profiles explain where the benchmark data comes from, how each problem is represented, what the project actually measured, and where the evidence stops.</p>
        <div class="dataset-index-stats" aria-label="Dataset inventory snapshot">
          <div><strong>{snapshot["entry_count"]:,}</strong><span>catalog entries</span></div>
          <div><strong>{snapshot["collection_count"]}</strong><span>source/family collections</span></div>
          <div><strong>{snapshot["observation_count"]:,}</strong><span>recorded observations</span></div>
          <div><strong>{snapshot["cohort_count"]}</strong><span>separate cohorts</span></div>
        </div>
        <p class="dataset-snapshot-note">Inventory snapshot: {_e(snapshot["inventory_date"])}. Campaign coverage: {_e(snapshot["evaluation_date"])}. Entries include mirrors, configurations, and variants; they are not all independent maps or measured workloads.</p>
      </header>

      <section class="dataset-reading-guide" aria-labelledby="dataset-reading-title">
        <div class="dataset-reading-heading">
          <p class="lesson-section-label">A FAIR COMPARISON HAS THREE LAYERS</p>
          <h2 id="dataset-reading-title">Read the problem before the score.</h2>
          <p>Dataset names tell you where an asset came from. They do not tell you which planner result is comparable.</p>
        </div>
        <div class="dataset-reading-grid">
          <article><span>01</span><h3>Problem semantics</h3><p>Graph or grid, state dimension, direction, movement rule, robot shape, cost units, and goal policy.</p></article>
          <article><span>02</span><h3>Map structure</h3><p>Free states, degree, components, clearance, obstacle arrangement, and graph-weight distribution.</p></article>
          <article><span>03</span><h3>Query distribution</h3><p>Which endpoints were selected, how far apart they are, whether they are reachable, and which source versions define them.</p></article>
        </div>
      </section>

      <section class="dataset-inventory-section" aria-labelledby="dataset-inventory-title">
        <div class="dataset-section-heading">
          <p class="lesson-section-label">THE COLLECTIONS</p>
          <h2 id="dataset-inventory-title">Choose a dataset profile.</h2>
          <p>Each page gives the source context, data semantics, evaluation protocol, measured coverage, and interpretation limits.</p>
        </div>
        <div class="dataset-profile-grid">{"".join(cards)}</div>
      </section>

      <section class="dataset-coverage-note">
        <p class="lesson-section-label">HOW TO READ THE COVERAGE</p>
        <h2>Catalogued does not mean evaluated.</h2>
        <p>The inventory lists 1,297 source entries across five source groups and 22 collections. The recorded campaign contains 226,255 observations across 12 separate cohorts, but coverage differs by source. BARN uses a derived point-robot model; OMPL resources were installed without planner runs; weighted terrain lacks its source cost table; and most large voxel maps were resource-limited.</p>
        <p>Keep each domain in its own cohort. A directed road graph, an eight-neighbor grid, a strict 26-neighbor voxel map, and a continuous robot configuration space do not share one objective or correctness oracle.</p>
        <div class="dataset-coverage-links">
          <a href="{_e(source_url(snapshot["characterization_path"]))}" target="_blank" rel="noreferrer">Read the dataset characterization report ↗</a>
          <a href="{_e(source_url(snapshot["installation_path"]))}" target="_blank" rel="noreferrer">Read the installation and provenance guide ↗</a>
        </div>
      </section>
    </main>
    <footer class="learning-footer">
      <span>Pathplanning Learning Lab</span>
      <a href="{asset_root}#evaluation">Evaluation method</a>
      <a href="{asset_root}#algorithms">Algorithm atlas</a>
      <a href="https://github.com/damminhtien/pathplanning" target="_blank" rel="noreferrer">Project source ↗</a>
    </footer>
  </body>
</html>
"""


def dataset_detail_page(
    dataset: dict[str, Any],
    datasets_by_id: dict[str, dict[str, Any]],
    snapshot: dict[str, Any],
    output_root: Path,
    output_file: Path,
) -> str:
    page_url = f"{BASE_URL}/datasets/{dataset['id']}/"
    title = f"{dataset['title']}: Data, Methods & Coverage | Pathplanning"
    description = dataset["description"]
    asset_root = _relative_root(output_root, output_file)
    hub_url = f"{asset_root}datasets/"
    section_links = "\n".join(
        f'<a href="#{_e(section["id"])}">{_e(section["title"])}</a>'
        for section in dataset["sections"]
    )
    sections_markup = "\n".join(_section(section) for section in dataset["sections"])
    related_markup = "\n".join(
        f'<a class="dataset-related-card" href="../{_e(related_id)}/">'
        f"<span>RELATED PROFILE</span><strong>{_e(datasets_by_id[related_id]['title'])}</strong>"
        f"<small>{_e(datasets_by_id[related_id]['tagline'])}</small></a>"
        for related_id in dataset["related"]
    )
    sources = _source_cards(dataset["sources"])
    files = _project_links(dataset["project_paths"])
    keywords = [
        dataset["title"],
        "robot motion planning benchmark",
        "path planning dataset",
        "autonomous robot evaluation",
        dataset["source_name"],
    ]
    schema_type = dataset["schema_type"]
    schema = json_ld(
        {
            "@context": "https://schema.org",
            "@type": schema_type,
            "name": dataset["title"],
            "description": description,
            "url": page_url,
            "sameAs": dataset["source_url"],
            "creator": {"@type": "Organization", "name": dataset["source_name"]},
            "inLanguage": "en",
            "keywords": keywords,
            "citation": [
                {"@type": "CreativeWork", "name": source["name"], "url": source["url"]}
                for source in dataset["sources"]
            ],
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
                {
                    "@type": "ListItem",
                    "position": 2,
                    "name": "Dataset guide",
                    "item": f"{BASE_URL}/datasets/",
                },
                {"@type": "ListItem", "position": 3, "name": dataset["title"], "item": page_url},
            ],
        }
    )
    head = _page_head(
        title=title,
        description=description,
        page_url=page_url,
        asset_root=asset_root,
        social_type="article",
        schema=schema,
        breadcrumb_schema=breadcrumb_schema,
    )
    return f"""{head}
  <body class="learning-body dataset-body">
    <a class="skip-link" href="#dataset-content">Skip to dataset profile</a>
    {_learning_nav(asset_root)}
    <main class="datasets-shell">
      {_breadcrumb([("Home", asset_root), ("Datasets", hub_url), (dataset["title"], None)])}
      <header class="dataset-detail-hero">
        <p class="lesson-kicker">{_e(dataset["eyebrow"])}</p>
        <h1>{_e(dataset["title"])}</h1>
        <p class="dataset-tagline">{_e(dataset["tagline"])}</p>
        <p class="dataset-intro">{_e(dataset["description"])}</p>
        <a class="dataset-source-pill" href="{_e(dataset["source_url"])}" target="_blank" rel="noreferrer">
          <span class="dataset-source-label">PRIMARY SOURCE</span>
          <span class="dataset-source-name">{_e(dataset["source_name"])}</span>
          <span class="dataset-source-arrow" aria-hidden="true">↗</span>
        </a>
        <div class="dataset-detail-stats">{_stat_cards(dataset["stats"])}</div>
        <p class="dataset-snapshot-note">Catalog snapshot: {_e(snapshot["inventory_date"])}. Campaign snapshot: {_e(snapshot["evaluation_date"])}. See the coverage section below for the exact denominator.</p>
      </header>

      <div class="dataset-detail-layout">
        <aside class="dataset-rail" aria-label="On this page">
          <p>ON THIS PAGE</p>
          {section_links}
          <a href="#sources">Primary sources</a>
          <a href="#project-files">Project implementation</a>
        </aside>
        <article id="dataset-content" class="dataset-content">
          {sections_markup}

          <section id="sources" class="dataset-section dataset-sources-section">
            <p class="lesson-section-label">PRIMARY REFERENCES</p>
            <h2>Go back to the source.</h2>
            <p>Use the original benchmark page and paper for source definitions, attribution, and version context. The project links below show how the repository discovers, parses, and evaluates these inputs.</p>
            <div class="dataset-source-grid">{sources}</div>
          </section>

          <section id="project-files" class="dataset-section dataset-project-section">
            <p class="lesson-section-label">REPRODUCIBLE PROJECT RECORD</p>
            <h2>Follow the data into the code.</h2>
            <p>These versioned files define the repository’s inventory, input contract, provenance, and measurement boundaries.</p>
            <div class="dataset-file-list">{files}</div>
          </section>

          <section class="dataset-related-section">
            <div class="dataset-related-heading">
              <p class="lesson-section-label">COMPARE THE RIGHT NEIGHBOR</p>
              <h2>Related dataset profiles</h2>
            </div>
            <div class="dataset-related-grid">{related_markup}</div>
            <a class="dataset-back-link" href="{hub_url}">Return to the complete dataset guide →</a>
          </section>
        </article>
      </div>
    </main>
    <footer class="learning-footer">
      <span>Pathplanning Learning Lab</span>
      <a href="{hub_url}">All dataset profiles</a>
      <a href="{asset_root}#evaluation">Evaluation method</a>
      <a href="https://github.com/damminhtien/pathplanning" target="_blank" rel="noreferrer">Project source ↗</a>
    </footer>
  </body>
</html>
"""
