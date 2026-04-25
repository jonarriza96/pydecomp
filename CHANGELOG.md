# Changelog

All notable changes to this project are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and this project
adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.1.5] - 2026-04-25
Fix wheel matrix: skip cibuildwheel test step (avoids `pycddlib` source build needing system cddlib), let macOS runners build only their native arch (`archs = ["auto"]`), and have `setup.py` add Homebrew/`/usr/local` include prefixes so `boost/geometry.hpp` is found.

## [1.1.4] - 2026-04-25
Fix wheel builds: enable EPEL for `eigen3-devel` on `manylinux_2_28`, drop unused `cddlib`/`lapack`/`blas`/`gmp` from `before-all` (only runtime deps), revert `project.license` to legacy form to avoid `packaging.licenses` import error in cp39 build env.

## [1.1.3] - 2026-04-25
Install Boost in CI (required by `boost/geometry.hpp` in the C++ extension); modernize `project.license` to SPDX form.

## [1.1.2] - 2026-04-25
Adding missing files for github workflow

## [1.1.1] - 2026-04-25
Modified `README.md`

## [1.1.0] - 2026-04-25

### Added
- `pyproject.toml` with PEP 517 build configuration and `[tool.cibuildwheel]` setup.
- GitHub Actions CI (`.github/workflows/ci.yml`) running build + smoke import on
  Python 3.9 through 3.13.
- Automated PyPI release pipeline (`.github/workflows/release.yml`) that builds
  manylinux + macOS wheels with `cibuildwheel` and publishes via PyPI Trusted
  Publishing (OIDC) on `v*` tag pushes, plus a GitHub Release with attached
  artifacts.
- `__version__` exposed from `pydecomp` as the single source of truth for the
  package version.
- `RELEASING.md` describing the tag-based release flow.

### Changed
- Relaxed dependency pins from exact equalities to lower bounds
  (`numpy>=1.23`, `scipy>=1.8`, etc.), enabling installs on Python 3.10+.
- `setup.py` now resolves `pybind11` headers via `pybind11.get_include()` and
  searches several common system locations for Eigen, so building from sdist
  no longer requires the vendored submodule or a hardcoded `/usr/include/eigen3`.
- Declared `requires-python = ">=3.9"` and expanded classifiers to cover
  Python 3.9–3.13.

### Removed
- Platform-specific `install_requires` branching in `setup.py`.

## [1.0.1]

- Previous release. See git history.
