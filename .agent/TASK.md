# Task: Publishable native wheels

## Goal

Provide tested binary wheels for the repository's evidenced CPython platforms
so ordinary installs do not need a local C/C++ toolchain.

## Scope

- Linux x86_64 using a manylinux_2_28 wheel (glibc 2.28+).
- macOS x86_64 and arm64 using native runners.
- CPython 3.10 through 3.14, standard GIL builds.
- Release workflow builds, validates, and publishes wheels together with the
  source distribution. Pull requests build and clean-install-test every wheel.

Windows, Linux arm64, musllinux, PyPy, and free-threaded CPython are excluded
until native builds and runtime behavior are validated for those targets.

## Acceptance criteria

- Each wheel is built in cibuildwheel's platform-compatible environment.
- Each wheel is installed in cibuildwheel's isolated test environment and a
  smoke check loads all packaged native libraries and runs a small public plan.
- Linux wheels use the manylinux_2_28 x86_64 baseline, which supports the
  current NumPy binary dependency wheels.
- Package metadata and installation documentation describe the tested wheel
  matrix and source-build requirements accurately.
- Pull request validation checks wheel and sdist metadata. PyPI publishing is
  gated to a published GitHub release or an explicit manual dispatch from
  `main`; manual publishing sends only wheel files.
- Local arm64 wheel build and clean installation pass. CI matrix supplies the
  Linux x86_64 and macOS x86_64 runtime validation.

## Constraints

- Preserve unrelated website work in the primary checkout.
- Do not create a new GitHub release. After the complete CI matrix passes,
  upload only wheel files to the existing PyPI release through the gated
  workflow dispatch from `main`.
- Stage only task-owned paths and commit the verified changes.
