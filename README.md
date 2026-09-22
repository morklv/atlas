# ATLAS

**A simulated drone survey that builds evidence maps and proposes routes through observed space.**

The first runnable release includes a geometric depth simulator, evidence mapping,
adaptive viewpoint selection, mission completion/return logic, three-policy
evaluation, an animated replay, and a guided learning plan.

A separate shared-map survey adds elevation/3D occupancy and coordinated
viewpoints. Its single [field workspace](output/swarm/field.html) combines
survey playback, local aerial semantic segmentation, a five-drone image-terrain
simulation, measured point-cloud import, LiDAR-semantic fusion primitives,
confidence-aware route inspection, ground-route planning and ROS-compatible
occupancy export. These are simulation and local-planning prototypes, not
autonomous real-world flight.

## Current implementation status

The working backend is a **lightweight kinematic sensor simulation**. It models
camera geometry, solid-obstacle occlusion, depth noise, missing data, movement,
geometric collision checks and a time budget. It runs on this Mac with NumPy and
Pillow. There are no hidden precomputed detections or obstacle coordinates in the planner.

**PX4/Gazebo flight integration is not complete or verified.** This Mac had no
working Xcode Command Line Tools, Homebrew or Gazebo at initial inspection.
The repository includes a Gazebo world exporter and separate telemetry/depth
probes. They are integration preparation, not evidence that the mission has
flown through PX4. See [the integration guide](docs/GAZEBO.md).

![Recorded simulated mission](output/demo/flight_replay.gif)

## Run it now

In Terminal:

```bash
cd /Users/mark/Documents/Atlas
bash run.sh demo
bash run.sh test
```

For the browser demonstration and local aerial-image inference, see
[the demo guide](docs/DEMO.md).

For the exact checks and their scope, see [the verification record](docs/VERIFICATION.md).

`run.sh` selects your project virtual environment, an installed Python with the
dependencies, or the available Codex Python runtime on Mark's Mac. It does not
install system software or modify your shell settings.

For a portable installation on another computer with Python 3.10+:

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -e .
.venv/bin/python -m sentinel demo
```

Each demo writes an animated mission replay, an observed-map report, compressed
depth frames, map snapshots, scene JSON, a Gazebo SDF world and an event log.
The default output directory is replaced on another run; choose `--out` to retain experiments.

```bash
bash run.sh demo --seed 42 --policy active --condition noisy --out output/my-experiment
bash run.sh demo --seed 42 --policy preplanned --condition noisy --out output/my-baseline
bash run.sh evaluate --seeds 100 101 102 --conditions nominal noisy intermittent
bash run.sh doctor
```

Open `output/demo/flight_replay.gif` to see simulated motion. Open
`output/demo/overview.png` for the observed map and latest depth frame.
The visualization is generated from recorded mission data, not a live flight UI.

## What the robot actually does

1. Starts at a known hover position, 7m above a 24m by 24m site.
2. Receives a 64 by 48 range image from a downward-tilted camera.
3. Projects valid ranges into world points using its known pose.
4. Adds endpoint evidence to 0.5m map cells: elevated surface, ground or unknown.
5. Predicts how much unknown space each candidate viewpoint might reveal.
6. Selects a view with useful predicted coverage and reasonable travel distance.
7. Moves, measures again and compares predicted gain with actual gain.
8. Returns to the starting hover when the observation limit, coverage target,
   time constraint or persistent sensor failure ends the mission.

Movement is at a constant height above the declared obstacle envelope. Takeoff,
landing, yaw dynamics and low-altitude collision avoidance are not simulated.

## Your engineering contribution to learn

The interesting code is in `mapping.py`, `planning.py` and `mission.py`:
sensor-to-map geometry, missing-data handling, viewpoint scoring, return-time
feasibility, and experiments. The initial implementation was produced with AI
assistance. Work through [the learning guide](docs/LEARNING.md), make changes and
record your own explanations before presenting it as demonstrated personal expertise.

## Three comparisons

| Policy | How it selects a view |
|---|---|
| `fixed` | A predetermined lawnmower sweep |
| `preplanned` | A greedy coverage route designed for an empty site before seeing sensor data |
| `active` | Recomputes the next view from the observed map after every inspection |

The stronger `preplanned` baseline matters: beating a short sweep alone can
overstate the benefit of feedback. Every run reports coverage, observed-cell
accuracy, obstacle recall including unknown cells, false-free cells, distance,
elapsed simulation time and return status.

See `output/evaluation/REPORT.md` for the development benchmark. The project
retains per-run JSON and CSV rather than reporting only favorable scenes.
These are small synthetic experiments, not flight or real-world validation.

A subsequent untouched-seed evaluation is in `output/heldout/REPORT.md` (seeds
200–209). Under nominal sensors, active averaged 92.8% observed coverage with
68.8m of travel, versus 91.3% and 94.4m for preplanned. Active had lower coverage
on two of those ten sites. These numbers describe this scene family and budget;
they are not a claim of general superiority.

## Repository map

```text
sentinel/
  geometry.py       Camera rays and coordinate conventions
  world.py          Hidden scene and ray intersections; evaluator/exporter
  sensor.py         Range camera, noise and dropout
  mapping.py        Endpoint evidence and predicted visibility
  planning.py       Fixed, preplanned and adaptive viewpoint policies
  mission.py        Mission execution and stop/return handling
  evaluate.py       Reproducible paired experiments
  render.py         Recorded map and flight visualizations
  lidar_fusion.py   Semantic and measured-height traversability fusion
  route_inspection.py  Uncertain-route task selection and swarm assignment
  ros2_ws/          Ubuntu ROS 2 Jazzy OccupancyGrid package
  integrations/     Optional PX4/Gazebo probes and ROS 2 point-cloud collector
tests/              Geometry, occlusion, budget, reproducibility and failure tests
docs/               Architecture, lessons, 14-day plan and integration status
```

## Scope and honest limitations

- Known exact pose: mapping, not SLAM or state-estimation research.
- Flat ground with static solid boxes; no vegetation, overhangs or moving objects.
- Range camera geometry; no learned computer vision, RGB segmentation or ML training.
- Near-boundary endpoints are discarded to reduce noisy cell assignments; this
  is a heuristic and is not a calibrated confidence estimate.
- Grid-aligned obstacle footprints simplify reference map labels.
- A time reserve approximates mission constraints; it is not a battery model.
- Straight-line movement at safe altitude; viewpoint selection is not a general
  obstacle-avoiding flight-path planner.
- Instant yaw and no wind, motor control, inertia or real sensor synchronization.
- Repeated noisy measurements are not statistically independent in a real camera;
  this simulator uses simplified random perturbations.

These limitations define the next learning and integration tasks. They should
remain visible in any portfolio description.
# Swarm field survey

Run `bash run.sh swarm` from this folder, then open `output/swarm/field.html` in a browser. This is the primary, unified interface. The recorded Python simulation has three drones sharing one height-evidence map and a ray-carved 0.5 m 3D occupancy grid. The page displays the survey and plans a ground route on the measured surface. `output/swarm/swarm.html` and `navigation.html` remain as legacy detailed views. `output/swarm/measured_points.ply` exports sampled range endpoints; `output/swarm/voxel_grid.npz` stores unknown/free/occupied volume; `output/swarm/swarm.json` contains the full replay data and metrics. Run `bash run.sh swarm-benchmark` for paired coordinated/independent trials over ten synthetic seeds. Individual comparison: `bash run.sh swarm --independent --out output/swarm-independent`.

This is a **synthetic 2.5D range-sensor experiment**. It does not contain a live LiDAR, radar, trained ML detector, SLAM, PX4 control, or real drone flight. Those are separate engineering milestones; the interface labels its reference terrain as simulator truth and keeps it distinct from the team's evidence map.

## Survey to route

`bash run.sh swarm` also writes `output/swarm/navigation.html`. It plans a
geometric ground or air route over the **observed** survey, with unknown space
blocked. You can import an XYZ CSV/ASCII PLY point cloud, choose robot limits and
start/destination, attach a photo for visual context, and optionally anchor the
local route to a known geographic origin for GeoJSON export. Pasting a Google
Maps coordinate or uploading one photo does not provide 3D terrain. An optional
ROS 2 PointCloud2 capture bridge is in the repository, but has not been run in a
ROS environment. See [navigation engineering status](docs/NAVIGATION.md) for
the data path, tests, limitations and next real-data experiment.

The new unified `field.html` adds a deliberately limited image mode: it turns
brightness from an uploaded overhead image into an **illustrative** height
hypothesis, simulates three overhead range observers scanning that hypothesis,
then applies the same observed-cell ground route planner. It never claims that
the image contains true elevation or that simulated scans validate the real
site. Measured XYZ/ASCII PLY data can instead be imported directly for routing;
that mode does not pretend the recorded swarm surveyed the imported points.
