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
- Narrowed the corrupt-tile-cache `except Exception` in
  `terraforge/data_acquisition/textures.py` to
  `(OSError, Image.UnidentifiedImageError)` so unexpected errors surface
  instead of being silently treated as a cache miss.
