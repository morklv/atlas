# ATLAS

ATLAS is a local terrain-perception and route-planning prototype. It uses a pretrained
PyTorch segmentation model to identify terrain classes in an aerial image, converts
those predictions into a traversability map, and uses A* search to calculate a
candidate rover route around predicted obstacles.

![ATLAS recorded synthetic terrain survey and candidate route](docs/images/atlas-preview.png)

*Recorded synthetic terrain survey with a candidate route. This is a simulation result.*

## What happens in the browser demo

1. You upload an aerial image.
2. A pretrained PyTorch model identifies terrain and creates a map of preferred and blocked areas.
3. The survey animation gradually reveals the map.
4. You choose two points. A* finds a candidate route, route inspection can check selected regions again, and the rover animation plays the route.

## Start the app on Mac

In Terminal, run:

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -e ".[field]"
bash run.sh swarm --out output/swarm
bash run.sh field-server --port 8767
```

Then open [http://127.0.0.1:8767](http://127.0.0.1:8767) in Safari.

## Optional: use an aerial image

To analyse an aerial image with a pretrained model, run:

```bash
.venv/bin/python -m pip install -e ".[vision]"
.venv/bin/python scripts/fetch_aerial_model.py
```

The model can label parts of an image, such as roads, trees, water, and buildings. It does not measure real height or prove that a route is safe.

The model is the pretrained `mfaytin/mask2former-satellite` Mask2Former checkpoint, run locally with PyTorch. ATLAS integrates inference; it does not train or fine-tune this model. Regional inspection runs inference again on image crops. Accuracy on a custom outdoor dataset has not been established.

## Check the code

Install Node.js to run the browser checks.

```bash
make test
make field
```

## Technology

- **Python** runs the backend, data processing, command-line tools, and tests.
- **PyTorch and Mask2Former** run pretrained semantic-segmentation inference locally.
- **FastAPI** serves the browser app and image-analysis endpoints.
- **JavaScript and HTML Canvas** provide the terrain view, survey animation, and route interaction.
- **A\*** searches the traversability grid for a candidate rover route.
- **NumPy** stores elevation, semantic, and traversability grids.
- **GitHub Actions** runs the automated Python, JavaScript, and C++ checks.

Other tested command-line tools in the repository use **Open3D** for point-cloud
processing, **SQLite** for experiment records, and **C++17** for elevation-grid
step and slope checks. They run separately from the browser demo.
