"""Renderer for one indexable algorithm lesson page."""

from __future__ import annotations

import html
from pathlib import Path
from typing import Any

from ..config import BASE_URL, FAMILY_LABELS
from ..content import source_url
from ..math_text import render_math_text
from .applications import render_lesson_applications
from .catalog import json_ld


def lesson_page(
    planner: dict[str, Any],
    lesson: dict[str, Any],
    analysis: dict[str, str],
    reference: dict[str, Any],
    registry_row: dict[str, str],
    applications: list[dict[str, Any]],
    planner_by_id: dict[str, dict[str, Any]],
    planner_positions: dict[str, int],
    total_planners: int,
    output_root: Path,
    output_file: Path,
) -> str:
    planner_id = planner["id"]
    title = planner["title"]
    kind = planner["kind"]
    page_url = f"{BASE_URL}/algorithms/{planner_id}/"
    description = (
        f"{title}: {FAMILY_LABELS[kind].lower()} path-planning lesson covering search mechanics, "
        "complexity, guarantees, application examples, trade-offs, and its research source."
    )
    depth = len(output_file.parent.relative_to(output_root).parts)
    asset_root = "../" * depth
    github_module = registry_row["module"].replace(".", "/") + ".py"
    module_url = source_url(github_module)
    reference_url = html.escape(reference["url"], quote=True)
    reference_note = (
        f'<p class="paper-note">{html.escape(reference["note"])}</p>'
        if reference.get("note")
        else ""
    )
    citation_text = (
        f"{reference['authors']} ({reference['year']}). {reference['title']}. {reference['venue']}."
    )
    related: list[str] = []
    for related_id in lesson["related"]:
        related_planner = planner_by_id[related_id]
        related.append(
            f'<a class="related-card" href="../{html.escape(related_id)}/">'
            f"<span>{html.escape(FAMILY_LABELS[related_planner['kind']])}</span>"
            f"<strong>{html.escape(related_planner['title'])}</strong>"
            f"<small>{render_math_text(related_planner['principle'])}</small></a>"
        )

    steps = "\n".join(
        f'<li><span class="lesson-step-number">0{index}</span><p>{render_math_text(step)}</p></li>'
        for index, step in enumerate(lesson["steps"], start=1)
    )
    example_steps = "\n".join(
        f'<li><span class="lesson-step-number">0{index}</span><p>{render_math_text(step)}</p></li>'
        for index, step in enumerate(lesson["example"]["steps"], start=1)
    )
    metric_tags = "\n".join(f"<li>{render_math_text(metric)}</li>" for metric in lesson["metrics"])
    schema = json_ld(
        {
            "@context": "https://schema.org",
            "@type": "LearningResource",
            "name": f"{title} Algorithm Analysis and Tutorial",
            "headline": title,
            "description": description,
            "url": page_url,
            "citation": {
                "@type": "CreativeWork",
                "name": reference["title"],
                "author": reference["authors"],
                "datePublished": str(reference["year"]),
                "url": reference["url"],
            },
            "inLanguage": "en",
            "learningResourceType": "Interactive tutorial",
            "isAccessibleForFree": True,
            "about": [
                {"@type": "Thing", "name": title},
                {"@type": "Thing", "name": "Autonomous agents"},
                {"@type": "Thing", "name": "Path planning algorithms"},
                {"@type": "Thing", "name": FAMILY_LABELS[kind]},
                {"@type": "Thing", "name": "Game AI and route planning"},
                {"@type": "Thing", "name": "Transportation and multi-agent planning"},
            ],
            "teaches": [
                "Path planning for autonomous agents",
                "Route planning across maps, graphs, and motion models",
                "Algorithm time and space complexity",
                planner["principle"],
            ],
            "educationalUse": "Instruction",
            "audience": {"@type": "EducationalAudience", "educationalRole": "student"},
            "keywords": [
                title,
                "path planning algorithm",
                "game AI navigation",
                "road network routing",
                "multi-agent pathfinding",
                "autonomous agents",
                "time complexity",
                "space complexity",
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
                    "name": "Algorithm atlas",
                    "item": f"{BASE_URL}/#algorithms",
                },
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
    <script>window.MathJax = {{tex: {{inlineMath: [["\\\\(", "\\\\)"], ["\\\\[", "\\\\]"]]}}}};</script>
    <script async id="mathjax-script" src="https://cdn.jsdelivr.net/npm/mathjax@4/tex-chtml.js"></script>
    <meta property="og:type" content="article" />
    <meta property="article:section" content="{html.escape(FAMILY_LABELS[kind], quote=True)}" />
    <meta property="og:site_name" content="Pathplanning Learning Lab" />
    <meta property="og:title" content="{html.escape(title)} Algorithm Analysis | Pathplanning" />
    <meta property="og:description" content="{html.escape(description, quote=True)}" />
    <meta property="og:url" content="{page_url}" />
    <meta property="og:image" content="{BASE_URL}/og-image.png" />
    <meta property="og:image:alt" content="An autonomous agent finding a route through obstacles" />
    <meta name="twitter:card" content="summary_large_image" />
    <meta name="twitter:title" content="{html.escape(title)} Algorithm Analysis | Pathplanning" />
    <meta name="twitter:description" content="{html.escape(description, quote=True)}" />
    <meta name="twitter:image" content="{BASE_URL}/og-image.png" />
    <meta name="twitter:image:alt" content="An autonomous agent finding a route through obstacles" />
    <script type="application/ld+json">{schema}</script>
    <script type="application/ld+json">{breadcrumb_schema}</script>
    <title>{html.escape(title)} Algorithm Analysis &amp; Tutorial | Pathplanning</title>
  </head>
  <body class="learning-body lesson-body family-{html.escape(kind)}">
    <a class="skip-link" href="#lesson-main">Skip to lesson</a>
    <header class="learning-topbar">
      <a class="brand" href="{asset_root}" aria-label="Pathplanning Learning Lab home">
        <span>path<span class="brand-accent">planning</span></span>
      </a>
      <nav aria-label="Learning navigation">
        <a href="{asset_root}#algorithms">Algorithm atlas</a>
        <a href="{asset_root}datasets/">Datasets</a>
        <a href="{asset_root}mindmap/">Project mind map</a>
        <a href="https://github.com/damminhtien/pathplanning" target="_blank" rel="noreferrer">Source</a>
      </nav>
    </header>
    <main id="lesson-main" class="lesson-shell">
      <nav class="breadcrumbs" aria-label="Breadcrumb">
        <a href="{asset_root}">Home</a><span aria-hidden="true">/</span>
        <a href="{asset_root}#algorithms">Algorithm atlas</a><span aria-hidden="true">/</span>
        <span aria-current="page">{html.escape(title)}</span>
      </nav>
      <header class="lesson-hero">
        <div class="lesson-kicker"><span class="family-dot"></span>{html.escape(FAMILY_LABELS[kind])} / LESSON {planner_positions[planner_id]:02d} OF {total_planners}</div>
        <h1>{html.escape(title)}</h1>
        <p class="lesson-tagline">{html.escape(lesson["tagline"])}</p>
        <p class="lesson-summary">{html.escape(planner["description"])}</p>
        <a class="lesson-principle" href="#mechanism"><span>The little rule</span><span class="lesson-principle-rule">{render_math_text(planner["principle"])}</span></a>
      </header>

      <div class="lesson-layout">
        <aside class="lesson-rail" aria-label="On this page">
          <p>ON THIS PAGE</p>
          <a href="#big-picture">The big picture</a>
          <a href="#watch">Watch it think</a>
          <a href="#mechanism">How it decides</a>
          <a href="#example">Worked example</a>
          <a href="#analysis">Algorithm analysis</a>
          <a href="#paper">Research paper</a>
          <a href="#tradeoff">Fit & trade-off</a>
          <a href="#evaluation">Evaluate fairly</a>
          <a href="#applications">Applications</a>
          <a href="#try-it">Your turn</a>
        </aside>
        <article class="lesson-content">
          <section id="big-picture" class="lesson-section">
            <p class="lesson-section-label">01. THE BIG PICTURE</p>
            <h2>A route is a chain of choices.</h2>
            <p>{render_math_text(lesson["story"])}</p>
          </section>

          <section id="watch" class="lesson-section lesson-watch">
            <div class="lesson-section-label">02. WATCH IT THINK</div>
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
            <p class="lesson-section-label">03. THE DECISION RULE</p>
            <h2>How does it decide what to do next?</h2>
            <p>{render_math_text(lesson["mechanism"])}</p>
            <h3 class="lesson-subheading">The search, in three moves</h3>
            <ol class="lesson-steps">{steps}</ol>
          </section>

          <section id="example" class="lesson-section lesson-example">
            <p class="lesson-section-label">04. WORK THROUGH AN EXAMPLE</p>
            <h2>Work through a small case.</h2>
            <p class="lesson-example-setup">{render_math_text(lesson["example"]["setup"])}</p>
            <ol class="lesson-steps lesson-example-steps">{example_steps}</ol>
          </section>

          <section id="analysis" class="lesson-section lesson-analysis">
            <p class="lesson-section-label">05. ALGORITHM ANALYSIS</p>
            <h2>What drives time, memory, and guarantees?</h2>
            <p class="analysis-intro">These bounds describe the planner's search work under its stated input model. Graph preparation, collision-check cost, and callback work can add time beyond the search-loop estimate.</p>
            <div class="analysis-complexity" aria-label="Complexity summary">
              <article class="analysis-complexity-card">
                <span>Runtime pattern</span>
                <span class="math-summary">{render_math_text(analysis["time_order"])}</span>
              </article>
              <article class="analysis-complexity-card">
                <span>Memory pattern</span>
                <span class="math-summary">{render_math_text(analysis["space_order"])}</span>
              </article>
            </div>
            <dl class="analysis-details">
              <div>
                <dt>Search state and ordering</dt>
                <dd>{render_math_text(analysis["model"])}</dd>
              </div>
              <div>
                <dt>Time complexity</dt>
                <dd>{render_math_text(analysis["time"])}</dd>
              </div>
              <div>
                <dt>Memory complexity</dt>
                <dd>{render_math_text(analysis["space"])}</dd>
              </div>
              <div>
                <dt>Guarantees and assumptions</dt>
                <dd>{render_math_text(analysis["guarantee"])}</dd>
              </div>
              <div>
                <dt>What changes as the problem grows</dt>
                <dd>{render_math_text(analysis["scaling"])}</dd>
              </div>
            </dl>
            <p class="analysis-source-note">Project-specific behavior: <a href="https://github.com/damminhtien/pathplanning/blob/main/SUPPORTED_ALGORITHMS.md" target="_blank" rel="noreferrer">supported planner contracts</a> and <a href="https://github.com/damminhtien/pathplanning/blob/main/docs/benchmark_contract.md" target="_blank" rel="noreferrer">benchmark and measurement method</a>.</p>
          </section>

          <section id="paper" class="lesson-section lesson-paper">
            <p class="lesson-section-label">06. THE RESEARCH SOURCE</p>
            <h2>Read the paper behind the idea.</h2>
            <p class="paper-intro">Start with the primary source for this planner. For older methods without one agreed origin paper, the reference is labeled as an early or foundational source.</p>
            <article class="paper-card">
              <div class="paper-card-meta">
                <span class="paper-source-label">{html.escape(reference["label"])}</span>
                <span>{html.escape(reference["authors"])} · {reference["year"]}</span>
              </div>
              <h3>{html.escape(reference["title"])}</h3>
              <p class="paper-venue">{html.escape(reference["venue"])}</p>
              {reference_note}
              <a class="paper-link" href="{reference_url}" target="_blank" rel="noopener noreferrer" aria-label="Read {html.escape(reference["title"], quote=True)}">Read the paper <span aria-hidden="true">↗</span></a>
            </article>
            <p class="paper-citation">{html.escape(citation_text)}</p>
          </section>

          <section id="tradeoff" class="lesson-section lesson-tradeoff">
            <p class="lesson-section-label">07. WHEN IT FITS</p>
            <h2>Choose it for the problem it solves.</h2>
            <div class="lesson-fit-grid">
              <article class="lesson-fit-card">
                <h3>A good fit</h3>
                <p>{render_math_text(lesson["best_for"])}</p>
              </article>
              <article class="lesson-fit-card lesson-fit-caution">
                <h3>Watch out</h3>
                <p>{render_math_text(lesson["watch_out"])}</p>
              </article>
            </div>
            <p class="lesson-project-tradeoff">In this project: {render_math_text(planner["tradeoff"])}</p>
            <div class="contract-note"><p>Algorithm guarantees depend on the graph, goal, costs, parameters, and stop reason. The project documents those exact conditions in its supported planner matrix.</p></div>
            <a href="https://github.com/damminhtien/pathplanning/blob/main/SUPPORTED_ALGORITHMS.md" target="_blank" rel="noreferrer">Read the supported contract</a>
          </section>

          <section id="evaluation" class="lesson-section lesson-evaluation">
            <p class="lesson-section-label">08. EVALUATE IT FAIRLY</p>
            <h2>What would make this a fair comparison?</h2>
            <p>{render_math_text(lesson["evaluation"])}</p>
            <h3 class="lesson-subheading">Useful measures for this planner</h3>
            <ul class="lesson-metrics">{metric_tags}</ul>
            <a class="lesson-method-link" href="{asset_root}#method">Explore this project's evaluation method</a>
          </section>

          <section id="applications" class="lesson-section lesson-applications">
            <p class="lesson-section-label">09. WHERE THE MODEL APPLIES</p>
            <h2>Where else does this planning model show up?</h2>
            <p>These examples share the same problem structure as this planner family. They are ways to transfer the idea, not claims that this project has deployed or benchmarked the application.</p>
            <div class="lesson-application-grid">{render_lesson_applications(applications)}</div>
          </section>

          <section id="try-it" class="lesson-section lesson-try">
            <p class="lesson-section-label">10. YOUR TURN</p>
            <h2>{html.escape(lesson["experiment"])}</h2>
            <details class="lesson-question">
              <summary>{render_math_text(lesson["question"])}</summary>
              <p>{render_math_text(lesson["answer"])}</p>
            </details>
          </section>

          <section class="lesson-source">
            <div><p class="lesson-section-label">IN THIS PROJECT</p><h2>One idea, one callable.</h2></div>
            <dl>
              <div><dt>Planner ID</dt><dd><code>{html.escape(planner_id)}</code></dd></div>
              <div><dt>Module</dt><dd><code>{html.escape(github_module)}</code></dd></div>
              <div><dt>Callable</dt><dd><code>{html.escape(registry_row["callable"])}</code></dd></div>
            </dl>
            <a class="button button-primary" href="{module_url}" target="_blank" rel="noreferrer">Read the implementation</a>
          </section>

          <section class="related-lessons">
            <p class="lesson-section-label">FOLLOW THE THREAD</p>
            <h2>Two nearby ideas</h2>
            <div class="related-grid">{"".join(related)}</div>
          </section>
        </article>
      </div>
    </main>
    <footer class="learning-footer">
      <a href="{asset_root}">Back to the learning lab</a>
      <a href="{asset_root}mindmap/">Explore the whole project map</a>
    </footer>
  </body>
</html>
"""
