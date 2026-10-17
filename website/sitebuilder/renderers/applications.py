"""Render application examples shared by the home page and planner lessons."""

from __future__ import annotations

import html
from typing import Any


def render_home_applications(applications: list[dict[str, Any]]) -> str:
    cards = []
    for application in applications:
        link = html.escape(application["link"], quote=True)
        cards.append(
            '<article class="application-card">'
            f'<p class="application-category">{html.escape(application["category"])}</p>'
            f'<h3>{html.escape(application["title"])}</h3>'
            f'<p class="application-description">{html.escape(application["description"])}</p>'
            '<div class="application-connection">'
            '<strong>Connection to this project</strong>'
            f'<p>{html.escape(application["project_note"])}</p>'
            '</div>'
            f'<a href="{link}">{html.escape(application["link_label"])} <span aria-hidden="true">→</span></a>'
            '</article>'
        )
    return "\n          ".join(cards)


def render_lesson_applications(applications: list[dict[str, Any]]) -> str:
    cards = []
    for application in applications:
        cards.append(
            '<article class="lesson-application-card">'
            f'<h3>{html.escape(application["title"])}</h3>'
            f'<p>{html.escape(application["scenario"])}</p>'
            '<p class="application-connection">'
            f'<strong>Planning connection.</strong> {html.escape(application["connection"])}'
            '</p>'
            '</article>'
        )
    return "\n                ".join(cards)
