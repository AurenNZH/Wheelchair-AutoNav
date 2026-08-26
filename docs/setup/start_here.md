# Start Here

## Build and test

```bash
cd /home/jetson-xavier-wheelchair/Wheelchair-AutoNav
git submodule update --init --recursive
source /opt/ros/foxy/setup.bash
cd ros2_ws
rosdep install --from-paths \
  src/wheelchair_bringup src/wheelchair_msgs src/wheelchair_navigation \
  src/wheelchair_shared_control src/wheelchair_simulation \
  src/unilidar_sdk2/unitree_lidar_ros2/src/unitree_lidar_ros2 \
  --ignore-src -r -y
colcon build --symlink-install \
  --packages-ignore unitree_lidar_ros unitree_lidar_sdk
source install/setup.bash
colcon test
colcon test-result --verbose
```

Run Raspberry Pi software tests separately with
`python3 -m pytest components/can_controller/tests`.

## Inspect L2/Nav2 without motion

Complete [L2 Ethernet preflight](l2_bringup.md), then run:

```bash
source /opt/ros/foxy/setup.bash
source ros2_ws/install/setup.bash
ros2 launch wheelchair_bringup wheelchair.launch.py \
  use_lidar:=true use_camera:=false use_mapping:=true use_rviz:=true
```

Verify both `/lidar_<side>/points` and `/lidar_<side>/points_filtered` streams,
both `/lidar_<side>/filter/source_header` heartbeats, and
`/nav2_front_costmap` independently.
No node in this launch commands the wheelchair.

## Run Gazebo before hardware

```bash
export ROS_DOMAIN_ID=91
ros2 launch wheelchair_simulation shared_control_sim.launch.py \
  gui:=false enable_sim_motion:=true operator_mode:=scenario scenario:=all
```

## Physical progression

Follow [shared_control_validation.md](shared_control_validation.md): validate
coverage and latency, measure swept geometry, test network failures, calibrate
weighted decisions, then use raised-wheel and lowest-speed controlled-area
tests with an independent cutoff. Never jump directly from RViz to an
obstacle-driving test.
