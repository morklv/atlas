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
2. Wait for local semantic segmentation. The map identifies road/track, open
   ground, low vegetation, forest, water, and buildings.
3. Run the eight-observer survey. Each observer contributes to a shared map.
4. Set a start and destination on traversable terrain, then plan a route.
5. Hover route-adjacent cells to inspect LiDAR evidence, confidence, terrain
   class, slope, and blocked reason.
6. Click **Request swarm route inspection**. The simulation targets uncertain
   route stretches, assigns nearby observers, adds evidence, and replans.
7. Click **Drive route** to show the planned route as a rover playback in the browser.
8. Download **Route + occupancy JSON**. It contains local ENU waypoints,
   route confidence, and OccupancyGrid-compatible values.

## Validate before a demo

```bash
make test
make field
```

The ROS 2 bridge is developed separately on Ubuntu. See `ros2_ws/README.md`.
The browser simulation does not command a vehicle, certify a route, or infer
measured depth from an aerial image.
