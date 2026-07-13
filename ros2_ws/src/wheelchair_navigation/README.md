# Wheelchair Navigation

ROS2 package for the current LiDAR-only local costmap and conservative motion
proposal prototype. It publishes debug outputs and never commands the
wheelchair.

## Build

```bash
cd ~/Wheelchair-AutoNav/ros2_ws
source /opt/ros/foxy/setup.bash
colcon build --symlink-install --packages-select wheelchair_navigation
source install/setup.bash
```

## Run

Define `base_link` at the ground projection of the wheelchair centre/rear-axle
midpoint, with X forward, Y left, and Z up. Publish the measured LiDAR pose:

```bash
ros2 run tf2_ros static_transform_publisher \
  X Y Z YAW PITCH ROLL base_link rslidar
```

Do not use a zero transform for clearance or trajectory validation. Start the
navigation node with:

```bash
ros2 launch wheelchair_navigation local_navigation.launch.py
```

Inputs and outputs:

- Input: `/rslidar_points`
- Target frame: `base_link`
- Costmap: `/local_costmap`
- Proposed command: `/proposed_cmd_vel`
- Selected path: `/local_planner/selected_path`

The proposed command is zero for invalid, empty, stale, future-dated, or
untransformable clouds, and when no sampled trajectory is collision-free.

In RViz, use `base_link` as the fixed frame and display `/local_costmap` as a
Map and `/local_planner/selected_path` as a Path.

## Test

```bash
colcon test --packages-select wheelchair_navigation
colcon test-result --verbose
```
