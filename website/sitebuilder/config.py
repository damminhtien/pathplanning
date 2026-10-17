"""Shared configuration for the static learning-site build."""

from pathlib import Path
import re

WEBSITE_ROOT = Path(__file__).resolve().parents[1]
PROJECT_ROOT = WEBSITE_ROOT.parent
BASE_URL = "https://damminhtien.github.io/pathplanning"

BUILD_MARKER = ".pathplanning-site-build"

PLANNER_ROWS = re.compile(
    r"^\| \x60(?P<kind>[^\x60]+)\x60 \| \x60(?P<id>[^\x60]+)\x60 \| "
    r"\x60(?P<module>[^\x60]+)\x60 \| \x60(?P<callable>[^\x60]+)\x60 \|",
    re.MULTILINE,
)

INLINE_TEX = re.compile(r"\\\((?P<inline>.+?)\\\)|\\\[(?P<display>.+?)\\\]", re.DOTALL)

FORMULA_PATTERNS = (
    re.compile(
        r"(?<![A-Za-z])f\s*\(\s*n\s*\)\s*=\s*g\s*\(\s*n\s*\)\s*\+\s*(?:w\s*[·*]\s*)?h\s*\(\s*n\s*\)"
    ),
    re.compile(r"(?<![A-Za-z])f\s*=\s*g\s*\+\s*h\b"),
    re.compile(r"(?<![A-Za-z])g\s*\+\s*w\s*[·*]\s*h\b"),
    re.compile(r"(?<![A-Za-z])g\s*\+\s*h\b"),
    re.compile(r"(?<![A-Za-z])g\s*/\s*rhs\b"),
    re.compile(r"(?<![A-Za-z])[fgh]\s*=\s*0\b"),
    re.compile(r"(?<![A-Za-z])[fgh]\s*=\s*\d+(?:\.\d+)?\b"),
    re.compile(r"(?<![A-Za-z])w\s*(?:>=|≤|<=|≥)\s*1\b"),
    re.compile(r"(?<![A-Za-z])w\s*[×*]\s*best\s*f\b"),
    re.compile(r"(?<![A-Za-z])n\s*\(\s*n\s*[−-]\s*1\s*\)\s*/\s*2\b"),
    re.compile(r"(?<![A-Za-z])V\s*=\s*W\s*[×*]\s*H\b"),
    re.compile(r"(?<![A-Za-z])E\s*≈\s*8V\b"),
    re.compile(r"(?<![A-Za-z])\|V\|\s*\^\s*A\b"),
    re.compile(r"(?<![A-Za-z])(?:A²T|n²)\b"),
    re.compile(r"(?<![A-Za-z])wᵢ\b"),
    re.compile(r"(?<![A-Za-z])[fgh]\s*\(\s*n\s*\)"),
    re.compile(
        r"(?<![A-Za-z])[fghw](?=\s+(?:value|values|is|does|estimate|estimates|"
        r"cost|costs|changes|from|for|and|already|keeps|records|decide|decides|"
        r"should|must|can|will|may|remains|remain|bound)\b)",
        re.IGNORECASE,
    ),
    re.compile(r"\b(?:estimate|track|multiply|heuristic)\s+(?P<math>[fgh])\b", re.IGNORECASE),
    re.compile(r"(?<![A-Za-z])rhs\b"),
)

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
