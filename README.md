# ATLAS

**Terrain perception and route planning for an outdoor rover prototype.**

ATLAS turns aerial imagery or imported point clouds into a conservative terrain map, plans an A* rover route through observed traversable space, and provides a ROS 2/Gazebo rover demonstration for the same navigation idea.

> This is a simulation and local-planning project. It is not validated on a physical robot, and an uploaded aerial image does not provide measured elevation or route clearance.

## What is implemented

- **Interactive field workspace:** a 3D browser view of an eight-observer aerial survey, terrain evidence, route confidence, and an animated ATLAS rover route playback.
- **Terrain perception:** optional local PyTorch/Mask2Former semantic segmentation maps aerial imagery to road, open ground, low vegetation, forest, water, and building classes.
- **Navigation:** traversability costs and A* planning block unknown, flooded, forested, and building cells by default.
- **Measured-data path:** XYZ CSV and ASCII PLY point-cloud import, slope-aware costs, and conservative LiDAR-semantic fusion primitives.
- **Robotics integration:** `ros2_ws/src/atlas_ros` contains a ROS 2 Jazzy package that publishes a seven-waypoint route, follows it with `/cmd_vel`, bridges odometry, and runs headlessly in Gazebo Harmonic.
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

## Enable local aerial-image segmentation

The model weights are intentionally excluded from Git because they are large. From the repository root:

```bash
python3 -m pip install -e ".[vision]"
python3 scripts/fetch_aerial_model.py
```

This downloads the public `mfaytin/mask2former-satellite` checkpoint to `.models/openearth-mask2former`. Inference is local and runs on CPU by default.

## ROS 2 rover demonstration

The ROS portion is developed for Ubuntu 24.04 with ROS 2 Jazzy and Gazebo Harmonic. It has been run headlessly in an Ubuntu ARM VM.

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch atlas_ros atlas_demo.launch.py
```

See [ros2_ws/README.md](ros2_ws/README.md) for the data flow and limitations.

## Architecture

```text
Aerial image / XYZ / PLY
        |
semantic labels + measured terrain evidence
        |
traversability cost map -> A* route -> browser rover playback
        |
ROS 2 Path -> route follower -> cmd_vel -> Gazebo rover -> odometry
```

## Verification

```bash
make test
make field
```

`make test` runs Python and browser-logic tests. `make field` rebuilds the local browser artifact and checks its inline JavaScript. GitHub Actions repeats these checks on Ubuntu.

## Honest boundaries

- The aerial model predicts **2D semantic classes**. It does not estimate surveyed elevation, vehicle clearance, or a safe real-world route.
- The browser rover is a visual playback of the planned route. It is not live-linked to Gazebo.
- The ROS/Gazebo demonstration runs headlessly because the ARM virtual machine's graphical Gazebo renderer was unstable.
- The point-cloud and ROS terrain-fusion code establish interfaces for real sensors; no physical LiDAR, GPS/IMU fusion, SLAM, edge deployment, or real-robot validation is claimed.

## Portfolio summary

Built ATLAS, a terrain-perception and rover-navigation prototype using Python, PyTorch, FastAPI, JavaScript, A* planning, ROS 2, Gazebo, and Linux. The project translates aerial semantic classes and point-cloud evidence into a conservative traversability map, plans rover routes through observed space, and validates a route-following data path in a ROS 2/Gazebo simulation.
