# Releasing pydecomp

Releases are cut by pushing a `vX.Y.Z` git tag. GitHub Actions builds wheels
across Linux + macOS for Python 3.9–3.13, builds the sdist, publishes to PyPI
via Trusted Publishing (OIDC, no token), and creates a GitHub Release with the
artifacts attached.

## Per-release steps

1. Bump `__version__` in `src/pydecomp/__init__.py`.
2. Add a section to `CHANGELOG.md` for the new version.
3. Commit:
   ```
   git commit -am "release vX.Y.Z"
   ```
4. Tag and push:
   ```
   git tag vX.Y.Z
   git push origin main --tags
   ```
5. Watch the **Release** workflow in the Actions tab. When green:
   - Wheels + sdist are on https://pypi.org/project/pydecomp/
   - A GitHub Release with the artifacts is created
6. Smoke-check in a fresh venv:
   ```
   pip install pydecomp==X.Y.Z
   python -c "import pydecomp; print(pydecomp.__version__)"
   ```

The release workflow includes a guard that fails the build if the git tag
(`vX.Y.Z`) does not match `__version__` in the package. Bump them together.

## One-time PyPI Trusted Publishing setup

This must be configured once on PyPI before the first OIDC release. On
https://pypi.org/manage/project/pydecomp/settings/publishing/ → **Add a trusted
publisher**:

- Owner: `jonarriza96`
- Repository: `pydecomp`
- Workflow name: `release.yml`
- Environment: `pypi`

The `release.yml` job uses `environment: pypi` and `permissions: id-token: write`,
which together let `pypa/gh-action-pypi-publish` mint a short-lived OIDC token
for PyPI — no API token in repo secrets is needed.

## Local verification before tagging

```
python -m pip install --upgrade build
python -m build --sdist           # MANIFEST.in regression check
pipx run cibuildwheel --platform linux   # full wheel matrix locally
```
