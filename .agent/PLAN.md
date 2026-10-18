# Plan: Native wheel publishing

| Milestone | Verification boundary | Status |
| --- | --- | --- |
| M1. Define supported wheel matrix | Repository evidence and native compiler flags establish target platforms and CPython versions. | done |
| M2. Add isolated wheel smoke check and CI/release workflow | cibuildwheel builds each target; each wheel is installed in its isolated test env and exercises native loading plus the public API. | done |
| M3. Update package metadata and install docs | README and package metadata state the tested wheel range and source-build fallback. | done |
| M4. Build, clean-install, inspect, and checkpoint | Local and remote wheel installs pass; the 15-wheel artifact set and sdist validate; merge, publish wheels to the existing version, and verify PyPI files. | done |
