"""Math-aware text formatting for statically rendered site content."""

from __future__ import annotations

import html
import re

from .config import FORMULA_PATTERNS, INLINE_TEX


def _big_o_spans(text: str) -> list[tuple[int, int, str]]:
    spans = []
    for match in re.finditer(r"\bO\s*\(", text):
        opening = text.find("(", match.start())
        depth = 0
        for index in range(opening, len(text)):
            if text[index] == "(":
                depth += 1
            elif text[index] == ")":
                depth -= 1
                if depth == 0:
                    spans.append((match.start(), index + 1, text[match.start() : index + 1]))
                    break
    return spans


def _latex_expression(expression: str) -> str:
    expression = re.sub(
        r"\|([A-Za-z])\|\s*\^\s*([A-Za-z0-9]+)", r"\\lvert \1\\rvert^{\2}", expression
    )
    expression = re.sub(r"\bbest\s+f\b", lambda _: r"f_{\mathrm{best}}", expression)
    expression = re.sub(r"\brhs\b", lambda _: r"\mathrm{rhs}", expression)
    return (
        expression.replace("·", r"\cdot ")
        .replace("×", r"\times ")
        .replace("−", "-")
        .replace("²", "^2")
        .replace("ᵢ", "_{i}")
        .replace("≥", r"\geq ")
        .replace(">=", r"\geq ")
        .replace("≤", r"\leq ")
        .replace("<=", r"\leq ")
        .replace("≈", r"\approx ")
    )


def render_math_text(value: str) -> str:
    """Escape plain copy and typeset recognized math as inline LaTeX."""
    spans: list[tuple[int, int, str, bool]] = []
    for match in INLINE_TEX.finditer(value):
        formula = match.group("inline") or match.group("display")
        spans.append((match.start(), match.end(), formula, match.group("display") is not None))

    protected = [(start, end) for start, end, _, _ in spans]
    candidates = _big_o_spans(value)
    for pattern in FORMULA_PATTERNS:
        for match in pattern.finditer(value):
            if "math" in match.groupdict():
                candidates.append((*match.span("math"), match.group("math")))
            else:
                candidates.append((match.start(), match.end(), match.group()))
    for start, end, formula in sorted(candidates, key=lambda item: (item[0], -(item[1] - item[0]))):
        if any(
            start < protected_end and end > protected_start
            for protected_start, protected_end in protected
        ):
            continue
        if any(
            start < existing_end and end > existing_start
            for existing_start, existing_end, _, _ in spans
        ):
            continue
        spans.append((start, end, _latex_expression(formula), False))

    spans.sort(key=lambda item: item[0])
    rendered = []
    cursor = 0
    for start, end, formula, display in spans:
        rendered.append(html.escape(value[cursor:start]))
        delimiter = (
            (r"\[" + html.escape(formula) + r"\]")
            if display
            else (r"\(" + html.escape(formula) + r"\)")
        )
        rendered.append(f'<span class="math-expression">{delimiter}</span>')
        cursor = end
    rendered.append(html.escape(value[cursor:]))
    return "".join(rendered)
