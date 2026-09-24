# ATLAS

ATLAS is a local browser demo for exploring terrain and planning a candidate rover route.

It runs on your computer. It is not a hosted website and it does not control a real vehicle.

## What works

- A browser workspace with a terrain map and 3D view.
- A simulated six-drone survey that progressively reveals the terrain map.
- A* route planning between two selected points.
- A route-corridor review that highlights uncertain areas and replans the candidate route.
- Import of local XYZ, CSV, TXT, or ASCII PLY point clouds for terrain review.
- Export of the planned route and occupancy grid as JSON.
- Automated Python and browser-logic tests on GitHub Actions.

## Run locally

### macOS or Linux

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -e ".[field]"
bash run.sh swarm --out output/swarm
bash run.sh field-server --port 8767
```

Then open [http://127.0.0.1:8767](http://127.0.0.1:8767) in a browser.

### Windows PowerShell

```powershell
py -m venv .venv
.\.venv\Scripts\python.exe -m pip install -e ".[field]"
.\.venv\Scripts\python.exe -m sentinel swarm --out output/swarm
.\.venv\Scripts\python.exe -m sentinel field-server --port 8767
```

Then open [http://127.0.0.1:8767](http://127.0.0.1:8767).

## Optional aerial-image analysis

ATLAS can run a pretrained local image-segmentation model after the optional model setup:

```bash
.venv/bin/python -m pip install -e ".[vision]"
.venv/bin/python scripts/fetch_aerial_model.py
```

The model predicts 2D terrain classes from an aerial image. The displayed terrain relief and drone survey remain illustrative; an uploaded image does not provide measured elevation or real-world route clearance.

## Verify the project

```bash
make test
make field
```

## Limits

- The drones and rover are browser simulations.
- A planned route is a candidate route for demonstration, not a safe route for a real robot.
- Imported point clouds are local files; ATLAS does not connect to live sensors.
