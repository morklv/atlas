# ATLAS

ATLAS is a local terrain-perception and rover-route planning demo. It analyzes an
aerial image with Mask2Former and PyTorch, converts the predicted terrain into a
weighted grid, and uses A* to calculate a route between two selected points.

![ATLAS terrain analysis and rover route](docs/images/atlas-preview.png)

## Demo workflow

1. Upload a PNG, JPEG, or WebP aerial image.
2. Run local semantic segmentation to create a terrain map.
3. Play the survey visualization as the map is revealed.
4. Select a start and destination and calculate an A* route.
5. Inspect selected route regions, update their predictions, and replan.
6. Play the rover route in the browser.

## Run on macOS

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -e ".[vision]"
.venv/bin/python scripts/fetch_aerial_model.py
make field
bash run.sh field-server --port 8767
```

Open [http://127.0.0.1:8767](http://127.0.0.1:8767) in Safari.

## Core technology

- **Python, PyTorch, Transformers, and Mask2Former** for local semantic segmentation
- **FastAPI and WebSockets** for the local application server and processing status
- **JavaScript and HTML Canvas** for terrain visualization and route interaction
- **A\*** and weighted terrain costs for rover-route planning
- **NumPy and Pillow** for image and grid processing
- **GitHub Actions** for automated Python and JavaScript checks

## Source code

- [`sentinel/field_server.py`](sentinel/field_server.py) — image inference and API
- [`sentinel/assets/field.html`](sentinel/assets/field.html) — browser interface and workflow
- [`sentinel/assets/navigation.js`](sentinel/assets/navigation.js) — terrain costs and A* routing
- [`sentinel/assets/field_core.js`](sentinel/assets/field_core.js) — survey and terrain processing
- [`sentinel/route_inspection.py`](sentinel/route_inspection.py) — route-region selection
- [`tests/`](tests/) — backend, terrain, and route-planning checks

## Verify

```bash
make test
make field
```
