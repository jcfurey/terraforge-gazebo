# Contributing to TerraForge Gazebo

Thanks for your interest in improving TerraForge Gazebo! This guide covers how
to set up a development environment, run the checks CI enforces, and submit
changes.

## Project layout

```
terraforge/            # Installed package (CLI, GUI, acquisition, processing, utils)
test/                  # pytest suite (functional + ament linters)
experimental/          # WIP widgets, NOT part of the installed package
launch/                # ROS 2 launch file (spawn_world.launch.py)
.github/workflows/     # CI (ros2_ci.yml)
```

## Development setup

The package is an `ament_python` ROS 2 package targeting **ROS 2 Jazzy** and
**Gazebo Harmonic** on Ubuntu 24.04. You can develop it either inside a ROS 2
workspace (recommended, matches CI) or as a plain Python package.

### Option A — ROS 2 workspace (matches CI)

```bash
mkdir -p ~/ws/src && cd ~/ws/src
git clone <your-fork-url> terraforge_gazebo
cd ~/ws
rosdep install --from-paths src --ignore-src -y
python3 -m pip install -r src/terraforge_gazebo/requirements.txt
colcon build --packages-select terraforge_gazebo
. install/setup.bash
```

### Option B — plain pip (CLI only)

`GDAL` and the GUI bits are the fiddly parts. The core CLI needs system
`libgdal-dev` matching the pip `GDAL` version (see the note in
`requirements.txt`).

```bash
python3 -m pip install -e .            # core CLI deps
python3 -m pip install -e .[gui]       # + PyQt6 GUI
python3 -m pip install -e .[experimental]  # + experimental map widget deps
```

## Running the checks

CI (`.github/workflows/ros2_ci.yml`) runs two test passes; reproduce them
locally before opening a PR.

**Ament linters** (copyright headers, PEP 8, docstring style), via colcon:

```bash
colcon test --packages-select terraforge_gazebo --event-handlers console_direct+
colcon test-result --verbose
```

**Functional pytest suite** (these don't register with colcon's test plugin,
so run them directly — exactly as CI does):

```bash
python3 -m pytest test/ \
    --ignore=test/test_copyright.py \
    --ignore=test/test_flake8.py \
    --ignore=test/test_pep257.py \
    -v --tb=short
```

Many tests `pytest.importorskip(...)` heavy geospatial deps (numpy, shapely,
GDAL, PIL), so they skip cleanly when a dep is missing rather than erroring.

## Coding conventions

- **Style:** PEP 8, enforced by `ament_flake8`. Keep lines within the
  project's existing width and import ordering.
- **Docstrings:** module- and public-function-level docstrings (PEP 257,
  enforced by `ament_pep257`). The codebase favors *why*-oriented comments —
  match that density.
- **Copyright headers:** test files carry the
  `# Copyright 2024 TerraForge Contributors` / `# Licensed under the MIT License.`
  header; follow the surrounding convention when adding files.
- **Coordinates:** all geospatial math runs in local UTM (never Web Mercator)
  so features align with terrain. Don't introduce Web-Mercator-space math in
  the placement path.

## Submitting changes

1. Branch off `master`.
2. Keep PRs focused; include tests for new behavior where practical.
3. Ensure both CI passes (linters + functional suite) are green locally.
4. Update `CHANGELOG.md` under the `Unreleased` heading.
5. Open a PR with a clear description of the change and its motivation.

## License

By contributing, you agree that your contributions are licensed under the
project's [MIT License](LICENSE).
