# State: Native wheel publishing

## Current status

M1 complete. The primary checkout has unrelated website edits; this work is
isolated in the managed `dev/native-wheels` worktree. Current host is macOS
arm64 with CPython 3.14.8. Docker is installed but its daemon is unavailable.

The repository's Ubuntu CI and current runner/toolchain evidence support Linux
x86_64; prior native builds cover macOS Intel and the active host covers macOS
arm64. `setup.py` uses GCC/Clang-style C11 and C++17 flags, so Windows/MSVC is
outside the validated scope. No Linux arm64 evidence was found.

## Current milestone

M2 is implemented, pending remote matrix validation. The targets are Linux
x86_64 manylinux2014, macOS x86_64, and macOS arm64; CPython 3.10-3.14.
cibuildwheel's isolated test environment runs a smoke script against each
wheel. M3 documentation and metadata changes are implemented.

## Validation evidence

- Repository branch starts clean at `dev/native-wheels`.
- `docker info` fails because no Docker daemon is running; local Linux wheel
  builds cannot run on this host. Linux and Intel macOS validation will be
  encoded in GitHub Actions but cannot be reported as executed until that
  workflow runs remotely.
- `cp314-macosx_arm64` built successfully and passed cibuildwheel's clean
  install test: all five native libraries loaded and the public A* plan
  succeeded. Wheel SHA-256: `6940c92c0cfecf2b42bd32444e13086401f6d9ed3b663d4f9275831b8c1c29d1`.
- A local all-Python macOS run stopped before compilation because CPython 3.10
  is not installed as a python.org framework; cibuildwheel does not install
  those system-wide interpreters outside CI. The full 3.10-3.14 matrix is
  configured for native GitHub runners.
- A synthetic filename fixture passed the 15-wheel matrix checker. The release
  workflow YAML parsed, and Linux cibuildwheel selected the five intended
  CPython build identifiers. Both helper scripts passed Python syntax, Ruff
  lint, and Ruff format checks. Local sdist and CPython 3.14 arm64 wheel both
  passed `twine check`; pyproject TOML parsing and `git diff --check` passed.

## Next action

Commit the verified task paths and open a PR so Linux x86_64 and macOS
Intel/arm64 clean-install matrix jobs can run. Review those remote results
before considering the existing-version PyPI wheel upload.
