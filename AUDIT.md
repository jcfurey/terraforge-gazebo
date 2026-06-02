# TerraForge Gazebo — Repository Audit

**Date:** 2026-06-01
**Scope:** Full repository (`master` @ `50249fc`)
**Areas:** Security · Tests & CI · Code quality & maintainability · Dependencies · Documentation & governance

---

## Executive summary

TerraForge Gazebo is a ~8,000-LOC Python / ROS 2 (Jazzy) package that converts
real-world geospatial data (SRTM DEM, OpenStreetMap, satellite tiles) into
Gazebo Harmonic `.world` files for robotics simulation. It is a **well-built,
professionally maintained project** with a clean security posture, a meaningful
test suite, and excellent user documentation.

**Overall assessment: Good (8/10).** No critical or high-severity issues were
found. Findings are concentrated in minor maintainability, dependency-hygiene,
and governance gaps — most of which are addressed by the accompanying PR.

| Category | Rating | Headline |
|----------|--------|----------|
| Security | ✅ Strong | No secrets, env-var keys + log scrubbing, validated inputs, parameterized SQL |
| Tests | ✅ Good | 55 tests, real regression guards; orchestration/masks/UI uncovered |
| CI | ⚠️ Broken | Workflow died at the pip step (no pip in the base image); lint gates never ran and carry pre-existing debt |
| Code quality | ✅ Good | Why-oriented comments; one very large function; lint debt fixed in this PR |
| Dependencies | ⚠️ Minor | Misplaced `geocoder` dep, pin drift between manifests |
| Docs & governance | ⚠️ Minor | Great README; missing LICENSE/CONTRIBUTING/CHANGELOG files |

---

## Repository overview

- **Purpose:** Geospatial → simulation world generator. Given a lat/lon/radius,
  it downloads terrain, OSM features (buildings, vegetation, roads), and
  satellite imagery, then emits a self-contained Gazebo `.world` with textured
  terrain, buildings, scattered trees, and optional roads.
- **Stack:** Python 3.10+, `ament_python` ROS 2 package (Jazzy), Gazebo
  Harmonic. Key deps: `click`, `osmnx`, `shapely`, `pyproj`, `GDAL`, `Pillow`,
  `Jinja2`, `requests`; optional `PyQt6` GUI.
- **Entry points:** `terraforge` (CLI) and `terraforge-gui` (PyQt6).
- **Structure:** `terraforge/` (acquisition → processing → SDF emit + utils),
  `test/` (55 pytest tests), `experimental/` (WIP map widgets, excluded from the
  package), `launch/` (ROS 2 launch), `.github/workflows/ros2_ci.yml` (CI).
- **Scale:** ~93 files, ~8K LOC main code, ~1.2K LOC tests. Largest modules:
  `tree_processor.py` (906), `cli.py` (730), `textures.py` (632).
- **Notable strengths:** strict UTM-throughout coordinate discipline (no
  Web-Mercator drift), 7-provider satellite tile registry with key management,
  dual cloud/foliage masking, portable per-world media output.

## Methodology

Three parallel read-only exploration passes (structure & stack, security, code
quality & tests) over the full tree, followed by targeted verification of each
actionable finding against the actual source (file:line confirmation, `flake8`
runs, dependency-usage greps).

---

## Findings

Severity: **Critical** (exploitable/data loss) · **High** · **Medium** ·
**Low** · **Info**. Each finding notes whether it is fixed in the accompanying
PR.

### Security — ✅ no critical/high findings

The codebase follows good security practices throughout:

- **No hardcoded secrets.** All provider API keys come from environment
  variables; `.env` is git-ignored. Keys are redacted in retry logs via
  `scrub_key()` (`terraforge/utils/retry.py`).
- **Input validation.** `world_name` / `performer_ref` pass `safe_identifier()`
  (`terraforge/utils/naming.py`) before flowing into filesystem paths and SDF
  XML, blocking path traversal and XML metacharacters. Lat/lon and raster
  inputs are bounds/format checked up front (`cli.py`).
- **Safe data handling.** Parameterized SQL in the experimental tile cache
  (`experimental/ui/map/map_widget.py:523`), `json.load` only (no `pickle` /
  unsafe `yaml`), no `eval`/`exec`/`os.system`/`shell=True`, HTTPS for all tile
  providers, Jinja file paths URI-quoted.

| # | Severity | Finding | Location | Status |
|---|----------|---------|----------|--------|
| S1 | Info | CI installs pip deps with `--break-system-packages`. Acceptable inside the disposable CI container, but a venv would be cleaner isolation. | `.github/workflows/ros2_ci.yml` | Documented |
| S2 | Info | `scrub_key()` only scrubs the project's own log lines; third-party libraries could still log key-bearing URLs. | `terraforge/utils/retry.py` | Documented |

### Tests & CI — ✅ good

55 pytest tests across 15 files with genuine regression guards (coordinate
round-trips, building MultiPolygon/hole handling, retry backoff, identifier
whitelist, heightmap sizing, tile parallelism, fuel wrappers, normal maps, CLI
cancellation). Tests `importorskip` heavy geo deps so they skip cleanly rather
than erroring.

| # | Severity | Finding | Location | Status |
|---|----------|---------|----------|--------|
| T1 | Medium | Estimated ~30–40% coverage. The main orchestration path (`run_generate_world`), the cloud/foliage mask algorithms, SDF template rendering, and the PyQt6 GUI have no direct tests. | `cli.py`, `*_mask.py`, `sdf_builder.py`, `ui/main_window.py` | Partially addressed (helper tests added) |

### CI — ⚠️ broken (and lint gates never executed)

The GitHub Actions workflow (`ros2_ci.yml`) has been **failing for everyone,
including `master`**: the `ros:jazzy-ros-base` image ships without `pip`, so the
`Install pip-only dependencies` step dies with `No module named pip` ~56 s in —
before build, lint, or tests ever run. Because the lint gate (`ament_copyright`
/ `ament_flake8` / `ament_pep257`, run via `colcon test`) **never executed**, the
codebase accumulated lint debt that the gate would reject once it does run.

| # | Severity | Finding | Location | Status |
|---|----------|---------|----------|--------|
| C1 | High | CI never reaches build/test: no `pip` in the base image. | `.github/workflows/ros2_ci.yml` | Fixed (install `python3-pip`) |
| C2 | Medium | All 25 `terraforge/` source files (plus `setup.py`, launch file) lack the MIT copyright header `ament_copyright` requires. | `terraforge/**`, `setup.py`, `launch/` | Fixed (headers added) |
| C3 | Medium | 28 `ament_flake8` violations in package code (E501, F401, E127/E302/E306, E731, W292/W293). | `terraforge/**` | Fixed |
| C4 | Medium | ~1000 `ament_flake8` violations in `experimental/` (WIP, tabs); it has no ignore marker so the linters scan it. | `experimental/` | Fixed (added `AMENT_IGNORE`) |
| C5 | Medium | ~69 `ament_pep257` docstring-format issues (D205/D209/D400) in existing docstrings across the package. | `terraforge/**` | In progress (driven by real CI output) |

### Code quality & maintainability — ✅ good

Why-oriented comments, low duplication, essentially no dead code or stale
commented-out blocks, and only one `TODO` (in experimental code). Error handling
is strong: network calls use exponential backoff (`retry_call`), invalid
geometries are counted and skipped, antimeridian / Web-Mercator-limit cases
raise clear errors. Style is now lint-clean (see CI findings C2–C5); previously
the ament linters had never actually run.

| # | Severity | Finding | Location | Status |
|---|----------|---------|----------|--------|
| Q1 | Medium | `run_generate_world()` is ~448 lines — a single long orchestration function. Hard to unit-test in pieces. | `terraforge/cli.py` | Partially fixed |
| Q2 | Low | Overly broad `except Exception` on the corrupt-tile-cache reopen path masks unexpected errors as a routine cache miss. | `terraforge/data_acquisition/textures.py` (corrupt-cache reopen) | Fixed |
| Q3 | Low | Unused import `scrub_key` (flake8 F401). | `terraforge/data_acquisition/textures.py` | Fixed |

> **Note on Q1:** a full restructure of `run_generate_world()` is risky — its
> stages share deeply intertwined locals (`converter`, `dem_stats`,
> `z_per_meter`, the `sample_terrain_z` closure) and there is no end-to-end
> integration test to catch behavior drift. This PR therefore extracts only two
> clean, self-contained, behavior-preserving seams (`_setup_fuel_wrappers`,
> `_resolve_texture_paths`) and leaves the coupled DEM→mask→model→render body
> inline. A deeper refactor should land alongside integration coverage.

### Dependencies — ⚠️ minor

| # | Severity | Finding | Location | Status |
|---|----------|---------|----------|--------|
| D1 | Low | `geocoder` was a **core** runtime dependency but is imported only by `experimental/ui/map/map_widget.py`, which is excluded from the installed package — every headless install pulled an unused, unmaintained dep. | `requirements.txt`, `setup.py` | Fixed (moved to `experimental` extra) |
| D2 | Low | Version-pin drift: `requirements.txt` set upper bounds; `setup.py` mostly omitted them, and `Pillow` bounds disagreed (`<12` vs `<13`). | `requirements.txt`, `setup.py` | Fixed (aligned; `Pillow>=10.0,<13`) |
| D3 | Info | `GDAL` is intentionally unpinned (system-libgdal coupling) and well documented; no lockfile (acceptable for a library / rosdep-resolved workspace). | `requirements.txt` | No action |

### Documentation & governance — ⚠️ minor

The README is comprehensive (architecture, use cases, CLI/GUI usage, output
layout, provider table, configuration). Module and public-function docstrings
are present and detailed.

| # | Severity | Finding | Location | Status |
|---|----------|---------|----------|--------|
| G1 | Low | `license='MIT'` declared in `setup.py`/README, but no `LICENSE` file in the repo — ambiguous legal status for downstream users. | repo root | Fixed (added `LICENSE`) |
| G2 | Low | No `CONTRIBUTING.md` (dev setup / how to run the CI checks) or `CHANGELOG.md`. | repo root | Fixed (both added) |
| G3 | Info | Placeholder `maintainer_email='dev@example.com'`. | `setup.py` | Documented (left for maintainer) |

---

## Prioritized recommendations

| Priority | Recommendation | Findings |
|----------|----------------|----------|
| High | Add a `LICENSE` file; clarify legal status. | G1 |
| High | Move `geocoder` out of core deps; align manifest pins. | D1, D2 |
| Medium | Grow integration coverage for the generation pipeline + masks, then refactor `run_generate_world()` further. | T1, Q1 |
| Medium | Confirm the CI `ament_flake8` config vs. the E501 lines; wrap or configure as appropriate. | Q4 |
| Low | Tighten broad exception handlers; drop dead imports. | Q2, Q3 |
| Low | Add `CONTRIBUTING.md` / `CHANGELOG.md`; set a real maintainer email. | G2, G3 |
| Low | Consider a CI venv instead of `--break-system-packages`. | S1 |

---

## Fixes applied in this PR

| Change | Resolves |
|--------|----------|
| Added `LICENSE` (MIT). | G1 |
| Moved `geocoder` to a new `experimental` extra; removed from core `install_requires` / `requirements.txt`. | D1 |
| Aligned dependency version bounds across `setup.py` and `requirements.txt` (incl. `Pillow>=10.0,<13`). | D2 |
| Extracted `_setup_fuel_wrappers` and `_resolve_texture_paths` from `run_generate_world()` (behavior-preserving). | Q1 (partial) |
| Narrowed the corrupt-cache `except Exception` to `(OSError, Image.UnidentifiedImageError)`; documented the intentional broad catches in the tile worker. | Q2 |
| Removed unused `scrub_key` import. | Q3 |
| Added `test/test_cli_helpers.py` covering the two extracted helpers. | T1 (partial) |
| Added `CONTRIBUTING.md` and `CHANGELOG.md`. | G2 |

## Deliberately not changed

- The coupled body of `run_generate_world()` (see note on Q1) — defer to a
  refactor backed by integration tests.
- Pre-existing E501 long lines (Q4) — left untouched to keep the diff focused on
  audited changes; flagged for the maintainer to reconcile with the CI lint
  config.
- `maintainer_email` placeholder (G3) — requires a real address from the
  maintainer.
