#!/usr/bin/env python3
"""Build GitHub Pages from the website's data and templates."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

from sitebuilder.config import WEBSITE_ROOT
from sitebuilder.pipeline import build


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=WEBSITE_ROOT / "dist")
    args = parser.parse_args()
    build(args.output)
    return 0


if __name__ == "__main__":
    sys.exit(main())
