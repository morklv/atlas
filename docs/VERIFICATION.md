# Verification record

Run from the repository root:

```bash
make test
make field
```

`make test` exercises Python geometry, mission, swarm, LiDAR-fusion, route-inspection, native-terrain, browser terrain, and route-planning checks. `make field` regenerates the browser artifact and checks its embedded JavaScript syntax. GitHub Actions repeats these checks on macOS with Python 3.11 and Node 22.

## Requirement evidence

| Project capability | Evidence |
|---|---|
| Aerial perception | Local FastAPI/PyTorch pretrained Mask2Former inference service and browser upload flow |
| Local field model | Browser survey playback and map-evidence tests |
| Point-cloud processing | Open3D cleaning, terrain grid, and SQLite experiment records |
| Terrain feasibility | C++17 step/slope tool with unknown cells blocked |
| Vehicle navigation | semantic costmap and A* browser tests |
| Reproducibility | Makefile and macOS GitHub Actions workflow |

The project has not been validated on a physical robot. Those are later deployment experiments, outside the evidence claimed by this repository.

## Local C++ build

`bash run.sh native-build` compiles the terrain tool. On macOS, an SDK/TAPI architecture mismatch triggers a retry with another installed Command Line Tools SDK. Other compiler errors remain errors. No system SDK is modified.

The test suite checks software behavior. It does not establish segmentation accuracy on real outdoor imagery. The Open3D, SQLite, fusion, and C++ components have separate Python/CLI checks; they are not all invoked by the browser upload flow.

## Verified locally on September 25, 2026

All 36 Python tests passed, including both native C++ tests, with no skips. All 16 JavaScript tests passed, and the field artifact generated and passed its syntax check. The recorded browser demo completed and produced a 33.3 m candidate route. This verifies software behavior, not physical route safety.
