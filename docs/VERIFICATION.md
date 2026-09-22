# Verification record

Run from the repository root:

```bash
make test
make field
```

`make test` exercises Python geometry, mission, swarm, LiDAR-fusion, route
inspection, ROS transform, browser terrain, and route-planning checks.
`make field` regenerates the browser artifact and checks its embedded JavaScript
syntax. The CI workflow repeats these checks on Ubuntu 24.04 with Python 3.11
and Node 22.

## Requirement evidence

| Project capability | Evidence |
|---|---|
| Aerial perception | Local FastAPI/PyTorch Mask2Former service and browser upload flow |
| Five-drone simulation | `simulateSurvey(..., 5)` and browser replay test |
| LiDAR fusion | `sentinel.lidar_fusion` tests for semantics, slope, unknown space and occupancy encoding |
| Uncertainty inspection | `sentinel.route_inspection` tests for task grouping and drone assignment |
| Vehicle navigation | semantic costmap and A* browser tests |
| ROS 2-ready output | browser route download contains OccupancyGrid-compatible values |
| ROS 2 bridge | `ros2_ws/src/sentinel_ros` PointCloud2-to-OccupancyGrid package |
| Reproducibility | Makefile and GitHub Actions workflow |

The simulator has not been validated on a real robot or a running ROS 2/Gazebo
installation. Those are deployment experiments, outside the evidence claimed
by this repository.
