"""Guard repository-wide lint coverage for supported code and config formats."""

from __future__ import annotations

from collections import Counter
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]

# A new source extension must be assigned to a checker before it can be merged.
SOURCE_SUFFIXES = {
    ".py",
    ".pyi",
    ".c",
    ".cc",
    ".cpp",
    ".cxx",
    ".h",
    ".hh",
    ".hpp",
    ".hxx",
    ".m",
    ".mm",
    ".js",
    ".jsx",
    ".mjs",
    ".cjs",
    ".ts",
    ".tsx",
    ".vue",
    ".svelte",
    ".css",
    ".scss",
    ".sass",
    ".less",
    ".html",
    ".htm",
    ".xml",
    ".svg",
    ".json",
    ".jsonc",
    ".yaml",
    ".yml",
    ".toml",
    ".sh",
    ".bash",
    ".zsh",
    ".fish",
    ".ipynb",
}

CHECKER_BY_SUFFIX = {
    ".py": "Ruff (whole repository)",
    ".pyi": "Ruff (whole repository)",
    ".js": "Biome (website)",
    ".css": "Biome (website)",
    ".html": "html-validate (website source and generated output)",
    ".c": "strict C11 compiler checks (pathplanning/native)",
    ".cc": "strict C++17 compiler checks (pathplanning/native)",
    ".cpp": "strict C++17 compiler checks (pathplanning/native)",
    ".cxx": "strict C++17 compiler checks (pathplanning/native)",
    ".h": "strict C11/C++17 header checks (pathplanning/native)",
    ".hh": "strict C++17 header checks (pathplanning/native)",
    ".hpp": "strict C++17 header checks (pathplanning/native)",
    ".hxx": "strict C++17 header checks (pathplanning/native)",
    ".xml": "XML well-formedness check",
    ".svg": "SVG/XML well-formedness check",
    ".json": "pre-commit JSON syntax check",
    ".yaml": "pre-commit YAML syntax check",
    ".yml": "pre-commit YAML syntax check",
    ".toml": "pre-commit TOML syntax check",
}

REQUIRED_ROOT_BY_SUFFIX = {
    ".js": "website/",
    ".css": "website/",
    ".html": "website/",
    ".c": "pathplanning/native/",
    ".cc": "pathplanning/native/",
    ".cpp": "pathplanning/native/",
    ".cxx": "pathplanning/native/",
    ".h": "pathplanning/native/",
    ".hh": "pathplanning/native/",
    ".hpp": "pathplanning/native/",
    ".hxx": "pathplanning/native/",
}


def repository_files() -> list[Path]:
    result = subprocess.run(
        ["git", "ls-files", "-z", "-c", "-o", "--exclude-standard"],
        cwd=ROOT,
        check=True,
        capture_output=True,
    )
    return sorted(Path(name.decode("utf-8")) for name in result.stdout.split(b"\0") if name)


def main() -> int:
    files = repository_files()
    code_files = [path for path in files if path.suffix.lower() in SOURCE_SUFFIXES]
    unsupported = [path for path in code_files if path.suffix.lower() not in CHECKER_BY_SUFFIX]
    misplaced = [
        path
        for path in code_files
        if path.suffix.lower() in REQUIRED_ROOT_BY_SUFFIX
        and not path.as_posix().startswith(REQUIRED_ROOT_BY_SUFFIX[path.suffix.lower()])
    ]

    if unsupported or misplaced:
        problems = [
            *(f"No checker for {path.as_posix()} ({path.suffix})" for path in unsupported),
            *(
                f"{path.as_posix()} is outside the configured {REQUIRED_ROOT_BY_SUFFIX[path.suffix.lower()]} scope"
                for path in misplaced
            ),
        ]
        print("Lint coverage is incomplete:\n" + "\n".join(problems), file=sys.stderr)
        return 1

    counts = Counter(path.suffix.lower() for path in code_files)
    summaries = [
        f"{suffix}={counts[suffix]} ({checker})"
        for suffix, checker in CHECKER_BY_SUFFIX.items()
        if counts[suffix]
    ]
    print(
        f"Lint coverage mapped for {len(code_files)} source/config files: " + "; ".join(summaries)
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
