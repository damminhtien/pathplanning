"""Small HTML components shared by the generated pages."""

from __future__ import annotations

import html
import json
from typing import Any

from ..config import FAMILY_LABELS
from ..math_text import render_math_text


def render_static_catalog(planners: list[dict[str, Any]]) -> str:
    cards = []
    for planner in planners:
        planner_id = html.escape(planner["id"], quote=True)
        title = html.escape(planner["title"])
        description = html.escape(planner["description"])
        kind = html.escape(FAMILY_LABELS[planner["kind"]])
        principle = render_math_text(planner["principle"])
        cards.append(
            f'<article class="planner-card"><div class="planner-card-top">'
            f'<span class="planner-family">{kind}</span></div>'
            f'<h3><a class="planner-card-title" href="algorithms/{planner_id}/">{title}</a></h3>'
            f'<p>{description}</p><div class="planner-card-footer">'
            f'<span class="planner-principle">{principle}</span>'
            f'<a class="card-lesson" href="algorithms/{planner_id}/">Read lesson</a>'
            f"</div></article>"
        )
    return "\n".join(cards)


def json_ld(value: dict[str, Any]) -> str:
    return json.dumps(value, ensure_ascii=False).replace("</", "<\\/")
