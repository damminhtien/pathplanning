# State: Native wheel publishing

## Current status

M1-M3 complete. The primary checkout has unrelated website edits; this work is
isolated in the managed `dev/native-wheels` worktree. Current host is macOS
arm64 with CPython 3.14.8. Docker is installed but its daemon is unavailable.

The repository's Ubuntu CI and current runner/toolchain evidence support Linux
x86_64; prior native builds cover macOS Intel and the active host covers macOS
arm64. `setup.py` uses GCC/Clang-style C11 and C++17 flags, so Windows/MSVC is
outside the validated scope. No Linux arm64 evidence was found.

## Current milestone

M2 and M3 are implemented and validated on PR #3. M4's local and remote wheel
builds, clean installs, and release-file validation are complete. The targets
are Linux x86_64 manylinux_2_28, macOS x86_64, and macOS arm64; CPython
3.10-3.14. Merge and the existing-version PyPI wheel upload remain.

## Validation evidence

- Task changes are isolated in `dev/native-wheels`; only task-owned paths are
  committed.
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
- First PR CI run: all standard lint, typecheck, unit, and slow-test jobs pass;
  macOS arm64 built and clean-installed all five CPython wheels. Linux built
  its wheel but clean installation tried to build NumPy 2.5.4 from source
  because the manylinux2014 environment had no compatible binary; NumPy's
  CPython 3.12 x86_64 wheel uses manylinux_2_27/2_28 and its source build needs
  GCC >= 10.3. The Linux baseline has been changed to manylinux_2_28. macOS
  Intel also completed successfully.
- PR run `39105446215` passes all three clean-install matrix jobs: Linux x86_64
  (6m32s), macOS arm64 (3m17s), and macOS x86_64 (10m06s). Each target built
  CPython 3.10-3.14 wheels, loaded all five native libraries, and ran the A*
  smoke plan. The distribution validation job passed `verify_wheelhouse.py`
  for 15 wheels plus the matching sdist, and `twine check` passed all files.
  The artifacts were downloaded and the 15-wheel matrix checker passed again
  against the actual filenames. Standard lint, Pyright, unit, and slow tests
  passed. The PyPI publish job was correctly skipped on the pull request.

## Next action

Merge PR #3, dispatch the wheel-only upload to the existing PyPI version from
`main`, then verify PyPI exposes all 15 wheels with the expected platform and
Python tags and perform a clean install from PyPI.
