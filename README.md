# TerraForge Gazebo

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-22314E.svg)](https://docs.ros.org/en/jazzy/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-orange.svg)](https://gazebosim.org/docs/harmonic)
[![Python](https://img.shields.io/badge/Python-3.10%2B-brightgreen.svg)](https://www.python.org/downloads/)

<p align="center">
  <img src="https://github.com/r3tr056/terraforge-gazebo/blob/master/.github/images/banner.png?raw=true" alt="TerraForge Gazebo">
</p>

Generate Gazebo Harmonic simulation worlds from real-world geospatial data. TerraForge downloads DEM elevation, OpenStreetMap building footprints, and Mapbox satellite tiles for a given lat/lon/radius, then emits a colcon-installable `ros2 launch`-able `.world` file.

## Status

Beta. Packaged as an **ament_python** ROS 2 package called `terraforge_gazebo`, targeting **ROS 2 Jazzy on Ubuntu 24.04** paired with **Gazebo Harmonic** (`gz-sim 8`).

## Key features

- **CLI and GUI**: generate worlds from the command line or from a small PyQt6 UI.
- **Real-world data**:
  - SRTM DEM via the `elevation` PyPI package -> 16-bit PNG heightmap.
  - OSM building footprints via `osmnx` -> per-building Gazebo models in local metric coordinates.
  - Mapbox satellite tiles -> PBR diffuse texture on the terrain.
- **Portable output**: each generated world ships with its own `models/` and `media/` subdirectories, loaded via `GZ_SIM_RESOURCE_PATH`.
- **ROS 2 integration**: a `spawn_world.launch.py` that wraps `ros_gz_sim`'s `gz_sim.launch.py`.

## Use inside a ROS 2 Jazzy workspace

```bash
# Prerequisites (once)
sudo apt install ros-jazzy-ros-gz python3-colcon-common-extensions \
    python3-rosdep python3-pip libgdal-dev

# Fetch
cd ~/ros2_ws/src
git clone https://github.com/r3tr056/terraforge-gazebo.git terraforge_gazebo

# Resolve ROS deps and pip-only deps
cd ~/ros2_ws
rosdep install --from-paths src -i -y
pip install -r src/terraforge_gazebo/requirements.txt

# Build + source
colcon build --packages-select terraforge_gazebo
source install/setup.bash

# Get a Mapbox token from https://account.mapbox.com/ and export it:
export MAPBOX_API_KEY='pk.xxx'

# Generate a world for downtown San Francisco, 500 m radius
terraforge generate-world \
    --latitude 37.7749 --longitude -122.4194 --radius 500 \
    --output-dir /tmp/sf --world-name sf

# Launch it in Gazebo Harmonic
ros2 launch terraforge_gazebo spawn_world.launch.py world:=/tmp/sf/sf.world
```

## Alternatively: standalone (no ROS)

```bash
git clone https://github.com/r3tr056/terraforge-gazebo.git
cd terraforge-gazebo
pip install -r requirements.txt
pip install -e .
export MAPBOX_API_KEY='pk.xxx'
terraforge generate-world --latitude 37.7749 --longitude -122.4194 \
    --radius 500 --output-dir /tmp/sf --world-name sf
# or launch the GUI
terraforge-gui
```

## Output layout

```
<output-dir>/
├── <world-name>.world
├── models/
│   ├── building_<osmid>_0/
│   │   ├── model.sdf
│   │   └── model.config
│   └── ...
└── media/
    ├── heightmap.png
    └── materials/
        └── textures/
            └── satellite_texture.png
```

The launch file prepends `<output-dir>/models` to `GZ_SIM_RESOURCE_PATH` so `<include><uri>model://building_...</uri></include>` resolves.

## Configuration

Environment variables:

| Variable | Default | Purpose |
| --- | --- | --- |
| `MAPBOX_API_KEY` | (required) | Mapbox access token for satellite tile downloads |
| `TERRAFORGE_CACHE_DIR` | `$XDG_CACHE_HOME/terraforge` or `~/.cache/terraforge` | Cache root for DEM/OSM/texture downloads |
| `TERRAFORGE_DEM_DIR` | `<cache-root>/dem` | Override DEM cache location |
| `TERRAFORGE_OSM_DIR` | `<cache-root>/osm` | Override OSM cache location |
| `TERRAFORGE_TEXTURE_DIR` | `<cache-root>/textures` | Override texture cache location |

## Modules

- `terraforge.cli` - `click`-based CLI, exposes `run_generate_world()` for programmatic use (e.g. from the GUI).
- `terraforge.data_acquisition` - DEM, OSM, Mapbox downloaders.
- `terraforge.data_processing` - DEM -> PNG, OSM -> SDF models, texture prep, Jinja SDF world template.
- `terraforge.utils.coordinates` - WGS84 <-> UTM <-> local-Gazebo conversions.
- `terraforge.ui.main_window` - PyQt6 GUI.
- `experimental/ui/` - Custom PyQt map/GL widgets. Not currently wired into the main GUI; kept for future map-preview work.

## License

MIT. See `LICENSE`.
