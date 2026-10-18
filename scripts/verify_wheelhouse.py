"""Check that a release contains exactly the supported native wheel matrix."""

from __future__ import annotations

from pathlib import Path
import sys

from packaging.utils import canonicalize_name, parse_sdist_filename, parse_wheel_filename
import tomllib

SUPPORTED_PYTHON_TAGS = {f"cp3{minor}" for minor in range(10, 15)}
SUPPORTED_ARCHITECTURES = {"linux-x86_64", "macos-x86_64", "macos-arm64"}
MANYLINUX2014_TAGS = {"manylinux2014_x86_64", "manylinux_2_17_x86_64"}


def wheel_architectures(platform_tags: set[str], filename: str) -> set[str]:
    architectures: set[str] = set()
    linux_tags = {tag for tag in platform_tags if tag.startswith("manylinux")}
    if linux_tags:
        if not linux_tags.intersection(MANYLINUX2014_TAGS) or not all(
            tag.endswith("_x86_64") for tag in linux_tags
        ):
            raise ValueError(f"{filename} has unsupported Linux platform tags: {linux_tags}")
        architectures.add("linux-x86_64")

    for tag in platform_tags - linux_tags:
        if tag.startswith("macosx_") and tag.endswith("_x86_64"):
            architectures.add("macos-x86_64")
        elif tag.startswith("macosx_") and tag.endswith("_arm64"):
            architectures.add("macos-arm64")
        else:
            raise ValueError(f"{filename} has an unsupported platform tag: {tag}")
    return architectures


def main() -> None:
    distribution_dir = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("dist")
    expected_name = canonicalize_name("pathplanning")
    project_metadata = tomllib.loads(Path("pyproject.toml").read_text(encoding="utf-8"))
    expected_version = project_metadata["project"]["version"]
    wheels = sorted(distribution_dir.glob("*.whl"))
    source_distributions = sorted(distribution_dir.glob("*.tar.gz"))

    expected = {
        (python_tag, architecture)
        for python_tag in SUPPORTED_PYTHON_TAGS
        for architecture in SUPPORTED_ARCHITECTURES
    }
    if len(wheels) != len(expected):
        raise ValueError(f"expected {len(expected)} wheels, found {len(wheels)}")

    actual: set[tuple[str, str]] = set()
    for wheel in wheels:
        name, version, _, tags = parse_wheel_filename(wheel.name)
        if canonicalize_name(name) != expected_name or str(version) != expected_version:
            raise ValueError(f"unexpected distribution identity in {wheel.name}")

        interpreter_tags = {tag.interpreter for tag in tags}
        if len(interpreter_tags) != 1:
            raise ValueError(
                f"expected one CPython interpreter tag in {wheel.name}: {interpreter_tags}"
            )
        interpreter_tag = next(iter(interpreter_tags))
        if interpreter_tag not in SUPPORTED_PYTHON_TAGS:
            raise ValueError(f"unsupported interpreter tag in {wheel.name}: {interpreter_tag}")

        architectures = wheel_architectures({tag.platform for tag in tags}, wheel.name)
        if not architectures or not architectures.issubset(SUPPORTED_ARCHITECTURES):
            raise ValueError(f"unsupported platform tags in {wheel.name}: {architectures}")
        for architecture in architectures:
            entry = (interpreter_tag, architecture)
            if entry in actual:
                raise ValueError(f"duplicate wheel for {entry}: {wheel.name}")
            actual.add(entry)

    if actual != expected:
        missing = sorted(expected - actual)
        extra = sorted(actual - expected)
        raise ValueError(f"wheel matrix mismatch; missing={missing}, extra={extra}")

    if len(source_distributions) != 1:
        raise ValueError(f"expected one source distribution, found {len(source_distributions)}")
    source_name, source_version = parse_sdist_filename(source_distributions[0].name)
    if canonicalize_name(source_name) != expected_name or str(source_version) != expected_version:
        raise ValueError(f"unexpected source distribution: {source_distributions[0].name}")

    print(
        f"Validated {len(wheels)} wheels for CPython 3.10-3.14 across "
        f"{', '.join(sorted(SUPPORTED_ARCHITECTURES))}, plus the matching sdist."
    )


if __name__ == "__main__":
    main()
