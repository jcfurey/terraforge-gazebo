# TerraForge Gazebo

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-22314E.svg)](https://docs.ros.org/en/jazzy/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-orange.svg)](https://gazebosim.org/docs/harmonic)
[![Python](https://img.shields.io/badge/Python-3.10%2B-brightgreen.svg)](https://www.python.org/downloads/)

<p align="center">
  <img src="https://github.com/r3tr056/terraforge-gazebo/blob/master/.github/images/banner.png?raw=true" alt="TerraForge Gazebo">
</p>

Generate Gazebo Harmonic simulation worlds from real-world geospatial data. TerraForge downloads a DEM, OpenStreetMap features (buildings, foliage, optional roads), and satellite-tile imagery for a given lat/lon/radius, then emits a colcon-installable `ros2 launch`-able `.world` file with a matching `models/` + `media/` tree.

## Status

Beta. Packaged as an **ament_python** ROS 2 package called `terraforge_gazebo`, targeting **ROS 2 Jazzy on Ubuntu 24.04** paired with **Gazebo Harmonic** (`gz-sim 8`).

## Key features

- **CLI and GUI**: generate worlds from the command line or from a small PyQt6 UI.
- **Real-world data**:
  - SRTM DEM via the `elevation` PyPI package → 16-bit PNG heightmap, resampled to Ogre2-valid `2^n+1` dimensions, vertically shifted so the world origin sits at real ground elevation.
  - OSM **buildings** via `osmnx` → per-building Gazebo models with SDF `<polyline>` footprint visuals + bbox collisions, height inferred from OSM tags, color by `building=*` category, base Z sampled from the DEM so buildings sit on slopes.
  - OSM **foliage** (`natural=tree`, `natural=wood`, `landuse=forest`) → trunk-cylinder + sphere-canopy tree instances, scattered inside forest polygons at a reproducible seeded density (clipped to the world bbox, capped at 200).
  - OSM **roads** *(opt-in, `--with-roads`)* → `highway=*` LineStrings buffered by per-class width into flat asphalt polyline ribbons.
  - **Cloud masking** on the satellite imagery — drops asset placements whose pixel looks cloud-like (high luminance + low saturation + morphological opening to dismiss small false-positive blobs like bright rooftops).
- **Seven satellite tile providers** with a registry (`esri`, `sentinel2`, `usgs_naip`, `gibs_bluemarble`, `mapbox`, `maptiler`, `bing`). Esri is the recommended keyless default. Mosaics are precision-cropped to the exact user bbox before saving.
- **Strict UTM coordinate handling** — bbox math and asset placement use the local UTM zone (EPSG:326XX / 327XX), not Web Mercator, so feature positions match the textured terrain to within pyproj precision at any latitude.
- **Portable output**: each generated world ships with its own `models/` and `media/` subdirectories, loaded via `GZ_SIM_RESOURCE_PATH`.
- **Physics-ready SDF template**: loads `bullet-featherstone`, the IMU + Contact systems, and a flat collision ground plane (since neither dartsim nor bullet-featherstone supports SDF heightmap collision). Workspace rovers drive on flat ground while the heightmap renders visually.
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

# Pick a satellite tile provider (see "Tile providers" below):
#   keyless:  esri | sentinel2 | usgs_naip | gibs_bluemarble
#   key req:  mapbox | maptiler | bing
export SATELLITE_TEXTURE_SOURCE=esri       # or pass --tile-provider on the CLI
# If you chose a key-required provider, also export its key, e.g.:
#   export MAPBOX_API_KEY='pk.xxx'         # mapbox
#   export MAPTILER_API_KEY='...'          # maptiler
#   export BING_MAPS_API_KEY='...'         # bing

# Generate a world for downtown San Francisco, 500 m radius
terraforge generate-world \
    --latitude 37.7749 --longitude -122.4194 --radius 500 \
    --output-dir /tmp/sf --world-name sf \
    --tile-provider esri

# Enumerate available providers + their attribution requirements
terraforge list-tile-providers

# Launch it in Gazebo Harmonic
ros2 launch terraforge_gazebo spawn_world.launch.py world:=/tmp/sf/sf.world
```

## Alternatively: standalone (no ROS)

```bash
git clone https://github.com/r3tr056/terraforge-gazebo.git
cd terraforge-gazebo
pip install -r requirements.txt
pip install -e .
export SATELLITE_TEXTURE_SOURCE=esri       # keyless default
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
│   ├── building_<osmid>_N/     (one per OSM building feature)
│   ├── tree_generic_0..4/      (5 reusable tree variants, <include>d many times)
│   └── road_<osmid>_N/         (only when --with-roads)
└── media/
    ├── heightmap.png           (16-bit, 2^n+1 sized)
    ├── cloud_mask.png          (debug: white = pixel was cloud-masked)
    └── materials/
        └── textures/
            └── satellite_texture.png
```

The launch file prepends `<output-dir>/models` to `GZ_SIM_RESOURCE_PATH` so `<include><uri>model://building_...</uri></include>` (and the tree / road includes) resolve.

## Configuration

Environment variables:

| Variable | Default | Purpose |
| --- | --- | --- |
| `SATELLITE_TEXTURE_SOURCE` | `mapbox` | Tile provider: `mapbox`, `esri`, `sentinel2`, `maptiler`, `bing`, `usgs_naip`, `gibs_bluemarble` |
| `MAPBOX_API_KEY` | "" | Access token for `mapbox` provider |
| `MAPTILER_API_KEY` | "" | Access token for `maptiler` provider |
| `BING_MAPS_API_KEY` | "" | Access token for `bing` provider |
| `TERRAFORGE_CACHE_DIR` | `$XDG_CACHE_HOME/terraforge` or `~/.cache/terraforge` | Cache root for DEM/OSM/texture downloads |
| `TERRAFORGE_DEM_DIR` | `<cache-root>/dem` | Override DEM cache location |
| `TERRAFORGE_OSM_DIR` | `<cache-root>/osm` | Override OSM cache location |
| `TERRAFORGE_TEXTURE_DIR` | `<cache-root>/textures` | Override texture cache location |

### Tile providers

| Provider | Key? | Max zoom | Coverage | Notes |
| --- | --- | --- | --- | --- |
| `mapbox` | required | 22 | global | Original default; highest fidelity |
| `esri` | no | 19 | global | Esri World Imagery; sub-meter in urban areas |
| `sentinel2` | no | 18 | global | Sentinel-2 cloudless 2023 via EOX (CC BY 4.0); 10 m native res |
| `maptiler` | required | 20 | global | Free tier ~100k tiles/mo |
| `bing` | required | 19 | global | QuadKey scheme; free tier ~125k tiles/yr |
| `usgs_naip` | no | 18 | US only | ~1 m aerial, public domain |
| `gibs_bluemarble` | no | 8 | global | NASA GIBS BlueMarble; very low res, useful only for wide-area backdrops |

Attribution is emitted as a log line per generation run; include it when publishing any resulting imagery.

## Modules

- `terraforge.cli` — `click`-based CLI. Exposes `run_generate_world()` for programmatic use (e.g. from the GUI or a wrapper script). Flags: `--tile-provider`, `--tile-api-key`, `--height-amplitude`, `--with-roads` / `--no-roads`, `--cloud-filter` / `--no-cloud-filter`.
- `terraforge.data_acquisition`
  - `elevation.py` — SRTM DEM download + **UTM-correct** WGS84 bbox calculation.
  - `osm.py` — buildings / foliage / roads downloaders (all `osmnx.features_from_bbox`).
  - `textures.py` — 7-provider tile registry, mosaic merge, exact-bbox crop.
- `terraforge.data_processing`
  - `elevation_processor.py` — DEM → normalized + Ogre2-resampled PNG; exposes `sample_dem_elevation(lat, lon)`.
  - `building_processor.py` — OSM polygons → per-building SDF models with polyline visuals + bbox collisions.
  - `tree_processor.py` — 5 reusable tree variants + forest-polygon scatter; clipped to world bbox.
  - `road_processor.py` — OSM LineStrings → polyline ribbons (visual-only).
  - `cloud_mask.py` — HLS-based cloud detection with morphological opening + per-(lat, lon) lookup.
  - `texture_processor.py` — copies the cropped mosaic into `media/materials/textures/`.
  - `sdf_builder.py` — Jinja SDF world template renderer.
  - `templates/world_template.sdf.j2` — the SDF skeleton (physics engine, lighting, terrain, buildings, trees, roads, collision ground-plane).
- `terraforge.utils.coordinates` — WGS84 ↔ UTM ↔ local-Gazebo converter (local UTM zone derived from origin longitude, not Web Mercator).
- `terraforge.ui.main_window` — PyQt6 GUI.
- `experimental/ui/` — Custom PyQt map/GL widgets. Not wired into the main GUI; kept for future map-preview work.

## License

MIT. See `LICENSE`.
