"""Render benchmark case studies for the home page and study index."""

from __future__ import annotations

import html
from pathlib import Path
from typing import Any

from ..config import BASE_URL
from .catalog import json_ld
from .datasets import _breadcrumb, _learning_nav, _page_head, _relative_root


def _e(value: str) -> str:
    return html.escape(value, quote=True)


def _visual(diagram: str) -> str:
    """Return a small, decorative SVG that previews each planning domain."""
    diagrams = {
        "grid": """<svg viewBox="0 0 320 150" aria-hidden="true" focusable="false">
          <path class="case-grid-lines" d="M0 25H320M0 50H320M0 75H320M0 100H320M0 125H320M25 0V150M50 0V150M75 0V150M100 0V150M125 0V150M150 0V150M175 0V150M200 0V150M225 0V150M250 0V150M275 0V150M300 0V150" />
          <path class="case-grid-walls" d="M73 25h28v25H73zM150 50h26v25h-26zM226 25h26v25h-26zM48 100h27v25H48zM200 100h25v25h-25z" />
          <path class="case-route" d="M25 137L25 87 62 87 112 62 137 62 137 37 187 37 187 87 212 87 262 62 300 12" />
          <circle class="case-node-start" cx="25" cy="137" r="6" /><circle class="case-node-goal" cx="300" cy="12" r="7" />
        </svg>""",
        "road": """<svg viewBox="0 0 320 150" aria-hidden="true" focusable="false">
          <g class="case-road-network"><path d="M20 126L70 92 128 109 176 72 230 82 300 28M20 126L58 36 128 109 140 20 230 82 254 130 300 28M58 36L140 20 176 72 254 130" /></g>
          <path class="case-route" d="M20 126L70 92 128 109 176 72 230 82 300 28" />
          <g class="case-road-nodes"><circle cx="20" cy="126" r="5" /><circle cx="58" cy="36" r="5" /><circle cx="70" cy="92" r="5" /><circle cx="128" cy="109" r="5" /><circle cx="140" cy="20" r="5" /><circle cx="176" cy="72" r="5" /><circle cx="230" cy="82" r="5" /><circle cx="254" cy="130" r="5" /><circle cx="300" cy="28" r="6" /></g>
        </svg>""",
        "continuous": """<svg viewBox="0 0 320 150" aria-hidden="true" focusable="false">
          <path class="case-contour" d="M10 26C56 45 30 78 70 112S124 125 145 85 188 27 225 50 276 112 310 120" />
          <path class="case-contour" d="M6 44C49 62 39 91 81 124M235 27C271 43 293 65 315 84" />
          <circle class="case-obstacle" cx="88" cy="48" r="17" /><circle class="case-obstacle" cx="165" cy="118" r="21" /><circle class="case-obstacle" cx="226" cy="27" r="14" />
          <path class="case-route" d="M18 132C48 124 53 85 83 84S120 107 137 72 171 54 194 71 222 99 247 87 280 55 304 17" />
          <circle class="case-node-start" cx="18" cy="132" r="6" /><circle class="case-node-goal" cx="304" cy="17" r="7" />
        </svg>""",
        "voxel": """<svg viewBox="0 0 320 150" aria-hidden="true" focusable="false">
          <g class="case-voxel-blocks">
            <path d="M53 56l18-10 18 10-18 11zM145 42l18-10 18 10-18 11zM230 57l18-10 18 10-18 11z" />
            <path d="M53 56v22l18 10V67M71 67l18-11v22L71 99M145 42v22l18 10V53M163 53l18-11v22l-18 10M230 57v22l18 10V68M248 68l18-11v22l-18 10" />
          </g>
          <path class="case-route" d="M20 132L75 132 118 126 164 130 210 127 268 109 294 30" />
          <circle class="case-node-start" cx="20" cy="132" r="6" /><circle class="case-node-goal" cx="294" cy="30" r="7" />
        </svg>""",
    }
    return f'<div class="case-study-visual case-study-visual-{diagram}">{diagrams[diagram]}</div>'


def render_home_case_studies(studies: list[dict[str, Any]]) -> str:
    cards = []
    for study in studies:
        stat = study["stats"][0]
        cards.append(
            f'<a class="case-study-card" href="case-studies/#{_e(study["id"])}">'
            f"{_visual(study['diagram'])}"
            f'<div class="case-study-card-copy">'
            f'<p class="case-study-kicker">{_e(study["eyebrow"])}</p>'
            f"<h3>{_e(study['title'])}</h3>"
            f'<p class="case-study-summary">{_e(study["summary"])}</p>'
            f'<div class="case-study-card-result"><strong>{_e(stat["value"])}</strong>'
            f"<span>{_e(stat['label'])}</span></div>"
            '<span class="case-study-card-link">Read the case study</span>'
            "</div></a>"
        )
    return "\n".join(cards)


def _stat_cards(stats: list[dict[str, str]]) -> str:
    return "\n".join(
        f'<div class="case-study-stat"><strong>{_e(stat["value"])}</strong>'
        f"<span>{_e(stat['label'])}</span></div>"
        for stat in stats
    )


def _findings(findings: list[dict[str, str]]) -> str:
    return "\n".join(
        f"<li><h3>{_e(item['title'])}</h3><p>{_e(item['text'])}</p></li>" for item in findings
    )


def _source_links(study: dict[str, Any], asset_root: str) -> str:
    dataset_url = f"{asset_root}datasets/{_e(study['dataset_id'])}/"
    items = [f'<a href="{dataset_url}">Dataset profile</a>']
    items.extend(
        f'<a href="{_e(source["url"])}" target="_blank" rel="noreferrer">{_e(source["label"])}</a>'
        for source in study["sources"]
    )
    return "".join(items)


def case_studies_page(studies: list[dict[str, Any]], output_root: Path, output_file: Path) -> str:
    page_url = f"{BASE_URL}/case-studies/"
    title = "Motion Planning Case Studies: Benchmarks, Methods & Findings | Pathplanning"
    description = (
        "Four evidence-led motion planning case studies across game maps, road networks, "
        "continuous geometry, and 3D voxel worlds. Read the protocol, results, and limits "
        "behind each benchmark claim."
    )
    asset_root = _relative_root(output_root, output_file)
    anchors = "".join(
        f'<a href="#{_e(study["id"])}">{_e(study["application"])}</a>' for study in studies
    )
    articles = []
    for index, study in enumerate(studies, start=1):
        articles.append(
            f'<article class="case-study-detail" id="{_e(study["id"])}" '
            f'aria-labelledby="{_e(study["id"])}-title">'
            f'<header class="case-study-detail-header">'
            f'<p class="case-study-kicker">CASE {index:02d} / {_e(study["eyebrow"])}</p>'
            f'<h2 id="{_e(study["id"])}-title">{_e(study["title"])}</h2>'
            f'<p class="case-study-application">{_e(study["application"])}</p>'
            f'<p class="case-study-lede">{_e(study["summary"])}</p>'
            "</header>"
            f'<div class="case-study-detail-grid">'
            f'<div class="case-study-evidence-column">'
            f"{_visual(study['diagram'])}"
            f'<div class="case-study-stats">{_stat_cards(study["stats"])}</div>'
            f'<nav class="case-study-source-links" aria-label="Sources for {_e(study["title"])}">'
            f"{_source_links(study, asset_root)}</nav>"
            "</div>"
            f'<div class="case-study-analysis-column">'
            f'<section class="case-study-question"><p class="case-study-section-label">THE QUESTION</p>'
            f"<p>{_e(study['question'])}</p></section>"
            f'<section class="case-study-protocol"><p class="case-study-section-label">EVALUATION PROTOCOL</p>'
            f"<p>{_e(study['protocol'])}</p></section>"
            f'<section class="case-study-findings"><p class="case-study-section-label">WHAT THE RESULTS SHOW</p>'
            f"<ol>{_findings(study['findings'])}</ol></section>"
            "</div></div>"
            f'<div class="case-study-reading-row">'
            f'<aside class="case-study-interpretation" aria-label="How to read: {_e(study["title"])}">'
            f"<h3>How to read this result</h3>"
            f"<p>{_e(study['interpretation'])}</p></aside>"
            f'<aside class="case-study-limitation" aria-label="Scope and limits: {_e(study["title"])}">'
            f"<h3>Scope and limits</h3>"
            f"<p>{_e(study['limitation'])}</p></aside>"
            "</div></article>"
        )

    schema = json_ld(
        {
            "@context": "https://schema.org",
            "@type": "CollectionPage",
            "name": "Motion Planning Case Studies",
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
                "numberOfItems": len(studies),
                "itemListElement": [
                    {
                        "@type": "ListItem",
                        "position": index,
                        "item": {
                            "@type": "TechArticle",
                            "headline": study["title"],
                            "description": study["summary"],
                            "url": f"{page_url}#{study['id']}",
                            "inLanguage": "en",
                            "about": [study["application"], study["dataset_id"]],
                        },
                    }
                    for index, study in enumerate(studies, start=1)
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
                {"@type": "ListItem", "position": 2, "name": "Case studies", "item": page_url},
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
        extra_stylesheets=("case-studies.css",),
    )
    return f"""{head}
  <body class="learning-body case-study-body">
    <a class="skip-link" href="#case-study-main">Skip to case studies</a>
    {_learning_nav(asset_root)}
    <main id="case-study-main" class="case-studies-shell">
      {_breadcrumb([("Home", asset_root), ("Case studies", None)])}
      <header class="case-studies-hero">
        <p class="lesson-kicker">MOTION PLANNING / EVIDENCE IN CONTEXT</p>
        <h1>Four worlds.<br /><span>Four planning questions.</span></h1>
        <p class="case-studies-lede">Based on the project benchmark refresh dated 10 October 2026, these studies keep the task model, denominator, validation rule, and limits in view. Read what each planner result can support, and where it stops.</p>
        <div class="case-studies-summary">
          <strong>{len(studies)} evidence-led studies</strong>
          <span>Game maps, road networks, continuous geometry, and 3D voxel worlds</span>
        </div>
      </header>
      <nav class="case-study-jumpnav" aria-label="Jump to a case study">{anchors}</nav>
      <div class="case-study-list">{"".join(articles)}</div>
      <section class="case-study-next-steps">
        <p class="lesson-section-label">CONTINUE THE INVESTIGATION</p>
        <h2>Explore the methods behind each result.</h2>
        <p>Use the dataset pages to inspect source semantics and coverage, then open the algorithm atlas for planner mechanics, implementation notes, and research references.</p>
        <div><a href="{asset_root}datasets/">Read the dataset guide</a><a href="{asset_root}#algorithms">Explore the algorithm atlas</a></div>
      </section>
    </main>
    <footer class="learning-footer">
      <span>Pathplanning Learning Lab</span>
      <a href="{asset_root}#evaluation">Evaluation method</a>
      <a href="{asset_root}datasets/">Dataset guide</a>
      <a href="https://github.com/damminhtien/pathplanning" target="_blank" rel="noreferrer">Project source</a>
    </footer>
  </body>
</html>
"""
