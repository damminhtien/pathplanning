# Task decisions: Native wheel publishing

## D-1: Publish only platforms with implementation evidence

- Decision: target Linux x86_64 manylinux2014 and macOS x86_64/arm64 wheels.
- Reason: Linux x86_64 is the CI target; native build records cover Intel
  macOS; the current Apple Silicon checkout provides arm64 build/test evidence.
  Windows/MSVC and Linux arm64 are not presently validated.

## D-2: Ship standard CPython wheels for declared interpreter support

- Decision: build CPython 3.10-3.14 wheels; skip free-threaded builds.
- Reason: Python metadata and project classifiers cover 3.10-3.13, and the
  current source build/test state records Python 3.14. No PyPy support is
  declared or tested.

## D-3: Exercise every wheel after installation

- Decision: use cibuildwheel's isolated per-wheel install/test environment to
  load every bundled native library and run a small public discrete plan.
- Reason: metadata-only wheel inspection would not detect missing native
  libraries, ABI loading errors, or a broken runtime link.

## D-4: Keep PyPI upload behind an explicit release gate

- Decision: pull requests build and validate artifacts. A published GitHub
  release uploads the sdist and wheels; a manual dispatch can upload only
  wheels to an existing release, and only when dispatched from `main` with the
  explicit publish input enabled.
- Reason: this repository already has a published sdist for 0.2.0, so adding
  wheels must not retry the immutable sdist filename. PyPI upload remains an
  external, irreversible versioned action.
