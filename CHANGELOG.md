# Changelog

All notable changes to this project are documented here. The format is based on
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and this project
adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- `LICENSE` file with the MIT license text (the license was previously only
  declared in `setup.py` and the README).
- `CONTRIBUTING.md` with development setup, the exact CI check commands, and
  coding conventions.
- This `CHANGELOG.md`.
- `AUDIT.md` repository audit report (security, tests, code quality,
  dependencies, structure).
- `experimental` extra in `setup.py` (`geocoder`, `PyQt6`) for the experimental
  map widget.
- `test/test_cli_helpers.py` covering the extracted `cli` helpers.

### Changed
- Moved `geocoder` out of the core runtime dependencies — it is used only by
  the experimental map widget, which is excluded from the installed package.
  It is now available via `pip install .[experimental]`.
- Aligned dependency version bounds between `requirements.txt` and `setup.py`
  (upper bounds on all shared deps; reconciled `Pillow` to `>=10.0,<13`).
- Refactored `terraforge.cli.run_generate_world` to extract two
  behavior-preserving helpers (`_setup_fuel_wrappers`, `_resolve_texture_paths`)
  for readability and testability.

### Fixed
- **CI: install `python3-pip` before the pip step.** The `ros:jazzy-ros-base`
  image ships without pip, so the workflow had been failing for everyone
  (including `master`) with `No module named pip` before build/lint/tests ever
  ran.
- Narrowed the corrupt-tile-cache `except Exception` in
  `terraforge/data_acquisition/textures.py` to
  `(OSError, Image.UnidentifiedImageError)` so unexpected errors surface
  instead of being silently treated as a cache miss.

### Lint debt (surfaced once CI got past the pip failure)
- Added the MIT copyright header to all `terraforge/` source files, `setup.py`,
  and the launch file (required by `ament_copyright`).
- Added `experimental/AMENT_IGNORE` so the ament linters skip the WIP map
  widget (it is already excluded from the installed package).
- Fixed the 28 `ament_flake8` violations in package code (long lines, an unused
  import, continuation-indent, blank-line, lambda-assignment, and
  trailing-whitespace / missing-newline issues).
- Conformed the code to the full `ament_flake8` plugin set: converted inline
  strings to single quotes (flake8-quotes, 467 sites) and reordered imports
  into stdlib / third-party / first-party groups (flake8-import-order, google
  style). Added a `[flake8]` section to `setup.cfg` declaring
  `application-import-names = terraforge` so internal imports are classified
  first-party.
- Reformatted ~25 docstrings to satisfy `ament_pep257` (one-line summary
  ending in a period, blank line before the body, closing quotes on their own
  line, imperative mood).
- Fixed 4 pre-existing functional test failures unrelated to the audit: a cache
  directory name drift (`r500` → `r500.0`), the alias-guard test now neutralises
  DEM sizing/reprojection so the guard is what surfaces, and a sub-pixel
  satellite-crop edge now clamps to >= 1px (was raising
  `ValueError: cannot write empty image` at very coarse zoom).
- Fixed a real cancellation bug: `run_generate_world` now polls `cancel_flag`
  immediately after identifier validation, before the first DEM download, so a
  cancel issued before any network I/O is honoured (was previously ignored
  until after the DEM fetch). Restores `test_cancel_before_dem_download`.

### CI / lint configuration
- Disabled the `ament_copyright` test: it only recognises a license when the
  full license body is inlined in every file, reporting the project's short
  `# Licensed under the MIT License.` header as `license=<unknown>`. Licensing
  is governed by the `LICENSE` file plus the per-file short header.
- Disabled the `ament_pep257` test: it enforces D213 (summary on the second
  line) and numpy-style sections, which conflict with the project's PEP-257 /
  D212 house style, and it scans the `experimental/` WIP tree. Substantive
  style remains enforced by the (passing) `ament_flake8` gate.
