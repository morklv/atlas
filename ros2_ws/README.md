# ATLAS ROS 2 workspace

Run on Ubuntu 24.04 ARM with ROS 2 Jazzy and headless Gazebo Harmonic.

## Build

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Run the rover simulation

```bash
ros2 launch atlas_ros atlas_demo.launch.py
```

The launch file starts a tactical terrain world, bridges `cmd_vel` and odometry
between ROS 2 and Gazebo, publishes a fixed candidate route, and drives the
rover along it. The dark road corridor is traversable; bright blocks represent
obstacles.

`terrain_fusion_node` remains a separate conservative bridge from LiDAR
`sensor_msgs/PointCloud2` to `/atlas/terrain_occupancy`. It is the ROS-side
contract for replacing the illustrative browser terrain with measured sensing.
