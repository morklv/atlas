# ATLAS demonstration

## Start the local workspace

```bash
cd /path/to/atlas
make field
bash run.sh field-server --port 8767
```

Open `http://127.0.0.1:8767/` in Safari. The local server is required for
aerial-image inference; opening `field.html` as a file will not provide it.

## Demonstrate the full simulated loop

1. Upload an overhead PNG, JPEG, or WebP image under 10 MB.
2. Wait for local pretrained semantic segmentation. The map identifies road/track, open
   ground, low vegetation, forest, water, and buildings.
3. Run the six-drone survey. Each observer contributes to a shared map.
4. Set a start and destination on traversable terrain, then plan a route.
5. Hover route-adjacent cells to inspect source-dependent evidence, heuristic confidence, terrain
   class, slope, and blocked reason.
6. Click **INSPECT ROUTE CORRIDOR →**. The simulation targets uncertain
   route stretches, assigns nearby observers, re-evaluates crops of the original image with the pretrained model, and replans.
   This can change predictions, but does not collect new sensor measurements or guarantee better accuracy.
7. Click **Drive route** to show the planned route as a rover playback in the browser.
8. Download **Route + occupancy JSON**. It contains local ENU waypoints,
   route confidence, and OccupancyGrid-compatible values.

## Validate before a demo

```bash
make test
make field
```

The browser workspace does not command a vehicle, certify a route, or infer
measured depth from an aerial image.

The default recorded synthetic survey uses three drones; aerial image uploads use six.
Install the optional vision dependencies and checkpoint using the README before uploading an image.
