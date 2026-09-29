# ATLAS demo

## Start

```bash
make field
bash run.sh field-server --port 8767
```

Open `http://127.0.0.1:8767/` in Safari.

## Demonstrate the workflow

1. Upload an overhead PNG, JPEG, or WebP image under 10 MB.
2. Wait for local Mask2Former segmentation to create the terrain grid.
3. Run the survey visualization.
4. Select a start and destination and plan an A* route.
5. Inspect the route to re-run segmentation on selected image regions and replan.
6. Play the rover route in the browser.

## Verify before recording

```bash
make test
make field
```
