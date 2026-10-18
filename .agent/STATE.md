# State: Native wheel publishing

## Current status

M1-M4 complete. PR #3 was squash-merged as `e690031b3d17becc74bddf7e9787e11350851d57`.
Workflow dispatch `39106842580` published the 15-wheel matrix to PyPI 0.2.0
without re-uploading its existing sdist. The primary checkout's unrelated
website changes were preserved; implementation work was isolated in the
managed `dev/native-wheels` worktree.

The repository's Ubuntu CI and current runner/toolchain evidence support Linux
x86_64; prior native builds cover macOS Intel and the active host covers macOS
arm64. `setup.py` uses GCC/Clang-style C11 and C++17 flags, so Windows/MSVC is
outside the validated scope. No Linux arm64 evidence was found.

## Released wheel matrix

The release supports CPython 3.10-3.14 on Linux x86_64 (manylinux_2_28), macOS
x86_64, and macOS arm64. Windows/MSVC, Linux arm64, musllinux, PyPy, and
free-threaded CPython remain outside the validated scope. The native build
uses GCC/Clang-style C11 and C++17 flags, so Windows/MSVC needs a separate,
validated compiler path before it can be added.

## Validation evidence

- Task changes were isolated in `dev/native-wheels`; only task-owned paths
  were committed.
- `docker info` fails because no Docker daemon is running; local Linux wheel
  builds cannot run on this host. GitHub Actions provides Linux and macOS
  validation.
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
- The first Linux CI attempt showed that manylinux2014 forced a source build
  of NumPy 2.5.4 because its CPython 3.12 x86_64 wheels use manylinux_2_27/2_28.
  Switching to manylinux_2_28 fixed the clean install.
- PR run `39106242067` passed all three clean-install matrix jobs: Linux x86_64
  (5m31s), macOS arm64 (3m12s), and macOS x86_64 (8m39s). Each target built
  CPython 3.10-3.14 wheels, loaded all five native libraries, and ran the A*
  smoke plan. `verify_wheelhouse.py` validated 15 wheels plus the matching
  sdist, `twine check` passed, and standard lint, Pyright, unit, and slow tests
  passed.
- Published workflow run `39106842580` passed the same three wheel build and
  clean-install jobs, release-file validation, and the PyPI upload job. The
  manual dispatch copied only `.whl` files, preserving the existing sdist.
- PyPI's simple index lists 15 wheels and the original sdist for 0.2.0. A fresh
  CPython 3.14 macOS arm64 virtual environment installed
  `pathplanning==0.2.0` with `--only-binary=:all:` and passed the five-library
  load plus public A* smoke check.

## Next action

No remaining release tasks. Add excluded platforms only after native build and
clean-install runtime validation exists for them.
