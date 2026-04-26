# Changelog

All notable changes to this project are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and this project
adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [2.0.0] - 2026-04-26

### Changed (breaking, at the dependency boundary)
- Cap `pycddlib<3`. The 3.x release rewrote its API (no more `cdd.Matrix(arr)` /
  `mat.rep_type = ...` / `cdd.Polyhedron(mat)`), which silently broke
  `_utils/visualize.py` and `_utils/environment.py` for users whose pip
  resolvers picked the latest pycddlib. The pydecomp public API is unchanged;
  the major bump is to mark the resolver-visible behavior change.

### Added
- `tests/` (pytest) covering import, public API surface, 2D decomposition
  shapes, and an end-to-end `visualize_environment` regression test that
  exercises the pycddlib code path. Wired into CI on Python 3.9–3.13.
- README: dedicated **Conda** section with a verified install recipe
  (conda-forge `cddlib`, `CPPFLAGS`/`LDFLAGS`, `tk` for matplotlib).

## [1.1.9] - 2026-04-26
Cap `pycddlib<3` (now superseded by the 2.0.0 entry).

## [1.1.8] - 2026-04-26
Drop `macos-13` (Intel) from the wheel matrix. Free-tier Intel macOS runners are scarce and unreliable (jobs sit in queue for 10+ minutes); Apple Silicon (`macos-14`) coverage plus the sdist fallback is enough.

## [1.1.7] - 2026-04-26
Pin `pypa/cibuildwheel@v2.23.4` (latest 2.x). Previous attempt to use the major-floating `v2` ref failed because the action publishes only specific point versions, not a `v2` tag.

## [1.1.6] - 2026-04-25
Bump `pypa/cibuildwheel` pin from `v2.21.3` → `v2`. The old pin tries to download virtualenv from a `github.com/pypa/get-virtualenv` URL that now returns HTTP 502, breaking macOS wheel builds (and stalling them on retry).

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
