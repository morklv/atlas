# ATLAS

**Terrain perception and route planning for an outdoor rover prototype.**

ATLAS turns aerial imagery or imported point clouds into a conservative terrain map and plans an A* rover route through observed traversable space.

> This is a simulation and local-planning project. It is not validated on a physical robot, and an uploaded aerial image does not provide measured elevation or route clearance.

## What is implemented

- **Interactive field workspace:** a 3D browser view of an eight-observer aerial survey, terrain evidence, route confidence, and an animated ATLAS rover route playback.
- **Terrain perception:** optional local PyTorch/Mask2Former semantic segmentation maps aerial imagery to road, open ground, low vegetation, forest, water, and building classes.
- **Navigation:** traversability costs and A* planning block unknown, flooded, forested, and building cells by default.
- **Measured-data path:** XYZ CSV and ASCII PLY point-cloud import, slope-aware costs, and conservative LiDAR-semantic fusion primitives.
- **Reproducible core simulation:** Python tests cover camera geometry, occlusion, evidence mapping, planning, route inspection, and browser planning logic.

## Run the field workspace

### macOS or Linux

```bash
python3 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -e ".[field]"
.venv/bin/python -m sentinel field-server --port 8767
```

Open [http://127.0.0.1:8767](http://127.0.0.1:8767). The server generates the browser artifact if it is missing.

### Windows PowerShell

```powershell
py -m venv .venv
.\.venv\Scripts\python.exe -m pip install --upgrade pip
.\.venv\Scripts\python.exe -m pip install -e ".[field]"
.\.venv\Scripts\python.exe -m sentinel field-server --port 8767
```

Open [http://127.0.0.1:8767](http://127.0.0.1:8767).

The browser survey, point-cloud import, route planning, and rover playback work with the `field` install. Image segmentation needs the optional model setup below.

## Process a measured point cloud

ATLAS can clean an ASCII PLY, XYZ, CSV, or TXT cloud with Open3D and export a conservative terrain grid. The result retains gaps as unknown cells and uses the highest measured point per grid cell.

```bash
python3 -m pip install -e ".[pointcloud]"
python3 -m sentinel pointcloud path/to/terrain.ply --out output/terrain --resolution 0.5
```

The command writes `terrain_grid.npz` and `report.json`. It does not invent missing terrain or claim a safe real-world route.

Each point-cloud run is also saved in `output/atlas_experiments.sqlite` with the input hash, settings, metrics, and artifact locations. Review recent runs with:

```bash
python3 -m sentinel experiments
```

## Native C++ terrain checks

ATLAS includes a small dependency-free C++17 core for checking a measured elevation grid against maximum step and slope limits. Unknown (`NaN`) cells stay blocked and the tool returns a machine-readable traversability mask.

```bash
python3 -m sentinel native-build
python3 -m sentinel native-terrain-cost elevation.npy --resolution 0.5 --max-step 0.25 --max-slope 20
```

The build uses Apple Clang, included with Xcode Command Line Tools. If the compiler reports an SDK or linker error, repair the local developer tools with `xcode-select --install`, then rerun it. This component checks geometric elevation limits only; it does not assess soil, water, wheel slip, or real-world safety.

## Enable local aerial-image segmentation

The model weights are intentionally excluded from Git because they are large. From the repository root:

```bash
python3 -m pip install -e ".[vision]"
python3 scripts/fetch_aerial_model.py
```

This downloads the public `mfaytin/mask2former-satellite` checkpoint to `.models/openearth-mask2former`. Inference is local and runs on CPU by default.

## Evaluate the segmentation model honestly

ATLAS includes a metric runner for your own labeled aerial images. Ground-truth grayscale masks use these values: `0` open, `1` road, `2` building, `3` forest, `4` low vegetation, and `5` water. Create a JSON manifest such as:

```json
[
  {"image": "orchard.png", "mask": "orchard_mask.png"}
]
```

Then run:

```bash
python3 -m sentinel evaluate-segmentation labels/manifest.json --out output/segmentation-evaluation
```

The report contains per-class precision, recall, IoU, pixel accuracy, and a confusion matrix. It does not claim results until real labeled imagery is provided.

## Architecture

```text
Aerial image / XYZ / PLY
        |
semantic labels + measured terrain evidence
        |
traversability cost map -> A* route -> browser rover playback
        |
native C++ step/slope feasibility mask for measured elevation grids
```

## Verification

```bash
make test
make field
```

`make test` runs Python and browser-logic tests. `make field` rebuilds the local browser artifact and checks its inline JavaScript. GitHub Actions repeats these checks on macOS.

## Honest boundaries

- The aerial model predicts **2D semantic classes**. It does not estimate surveyed elevation, vehicle clearance, or a safe real-world route.
- The browser rover is a visual playback of the planned route; it does not control a physical robot.
- The point-cloud path handles local files. No physical LiDAR, GPS/IMU fusion, SLAM, edge deployment, or real-robot validation is claimed.

## Portfolio summary

Built ATLAS, a Mac-native terrain-perception and rover-route-planning prototype using Python, PyTorch, FastAPI, JavaScript, Open3D, SQLite, C++, and A* planning. The project translates aerial semantic classes and point-cloud evidence into a conservative traversability map and plans candidate rover routes through observed space.
