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
| Aerial perception | Local FastAPI/PyTorch Mask2Former service and browser upload flow |
| Local field model | Browser survey playback and map-evidence tests |
| Point-cloud processing | Open3D cleaning, terrain grid, and SQLite experiment records |
| Terrain feasibility | C++17 step/slope tool with unknown cells blocked |
| Vehicle navigation | semantic costmap and A* browser tests |
| Reproducibility | Makefile and macOS GitHub Actions workflow |

The project has not been validated on a physical robot. Those are later deployment experiments, outside the evidence claimed by this repository.
