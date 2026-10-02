# Support

## Getting help

- Check `README.md` and `SUPPORTED_ALGORITHMS.md` for usage and supported surface.
- File issues on GitHub with clear repro steps and logs.
- For security concerns, follow `SECURITY.md`.

## Before filing an issue

- Reproduce on the latest `main`.
- Install with `pip install .` or run `make build-ext` in a checkout so the C and C++ libraries are present.
- Run `ruff check .` and `make test` to ensure the failure isn’t from local changes.
- Include environment details (OS, Python version, package version).
- For native build or load failures, include the compiler name/version and the full build error. See `docs/native_core.md` for the library split and ABI boundary.
