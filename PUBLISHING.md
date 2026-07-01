# Publishing `far-unitree-sdk` to PyPI

Wheels are built and published by GitHub Actions
([`.github/workflows/release.yml`](.github/workflows/release.yml)) using
[cibuildwheel](https://cibuildwheel.pypa.io) and **Trusted Publishing** (OIDC).
No API tokens are stored in the repo.

- **Distribution name:** `far-unitree-sdk` (the `far-` prefix avoids colliding
  with any upstream Unitree project).
- **Import name (unchanged):** `import unitree_interface`.
- **Wheels:** `manylinux_2_31` for Python 3.8 / 3.9 / 3.10 / 3.11 on x86_64 +
  aarch64, with the FastDDS runtime libraries bundled in by `auditwheel`.

## Build runners (AWS CodeBuild)

Both the build and test jobs run on native per-arch AWS CodeBuild runners — no
QEMU, and aarch64 is built and import-tested natively:

- `sdk-runner-unitree-x86` → x86_64 jobs
- `sdk-runner-unitree-arm` → aarch64 jobs

These must be CodeBuild **Runner projects** (runner provider GitHub, with a
`WORKFLOW_JOB_QUEUED` webhook) and have **Docker available** (cibuildwheel
launches manylinux containers, and the test job runs `python:X-slim`). The
workflow references them via the
`codebuild-<project>-${{ github.run_id }}-${{ github.run_attempt }}` label
convention. If you rename the runner projects, update `runs-on:` in
`.github/workflows/release.yml`.

## One-time setup: register the Trusted Publisher

The workflow's `publish` job targets **TestPyPI** first (see "Going to real
PyPI" below to switch). Trusted Publishing must be registered before the first
run — do this once per index.

### TestPyPI

1. Sign in at <https://test.pypi.org>.
2. Go to **Account settings → Publishing → Add a new pending publisher**
   (a "pending" publisher lets you claim a project name that doesn't exist yet).
3. Fill in **exactly**:
   - **PyPI project name:** `far-unitree-sdk`
   - **Owner:** `amazon-far`
   - **Repository name:** `unitree_sdk2`
   - **Workflow name:** `release.yml`
   - **Environment name:** `testpypi`
4. Save.

### Create the matching GitHub environment

1. In the GitHub repo: **Settings → Environments → New environment**, name it
   `testpypi` (must match the `environment:` in `release.yml` and the publisher
   registration above).
2. No secrets are needed — OIDC handles auth. Optionally add required reviewers
   to gate publishes.

## Cut a release

```bash
# Make sure pyproject.toml `version` is what you want to publish, then:
git tag v0.1.4
git push origin v0.1.4
```

Pushing a `v*` tag triggers the workflow: it builds the full wheel matrix, then
the `publish` job uploads to PyPI. A **manual** run (Actions → Run workflow)
builds wheels only and does **not** publish — useful for smoke-testing.

## Verify from TestPyPI

```bash
python -m venv /tmp/venv && source /tmp/venv/bin/activate
pip install --index-url https://test.pypi.org/simple/ far-unitree-sdk
python -c "import unitree_interface; print('ok')"
```

## Going to real PyPI

After TestPyPI looks good, switch the `publish` job in `release.yml`:

1. Register a Trusted Publisher on <https://pypi.org> with the same details, but
   use environment name `pypi`.
2. Create a `pypi` GitHub environment.
3. In `release.yml`, change `environment: testpypi` → `environment: pypi` and
   **remove** the `repository-url:` line (it defaults to real PyPI).

> PyPI uploads are immutable: a version number can never be reused, even if
> deleted. Bump `version` in `pyproject.toml` for every release.
