# Wheelchair Bringup

Top-level ROS2 launch package for repeatable sensor and navigation testing. It
does not command the wheelchair.

## Build and Source

The AIRY driver lives in a separate workspace, which must be sourced before
this repository workspace:

```bash
source /opt/ros/foxy/setup.bash
source ~/lidar_workspace/install/setup.bash
cd ~/Wheelchair-AutoNav/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

A launch file cannot source another workspace on your behalf.

## Launch

Start both sensors and publish the marked `rslidar -> camera_link` transform:

```bash
ros2 launch wheelchair_bringup wheelchair.launch.py
```

Add the installed RViz view:

```bash
ros2 launch wheelchair_bringup wheelchair.launch.py use_rviz:=true
```

Navigation is disabled by default because the real `base_link -> rslidar`
transform has not been measured. After measuring it, start navigation and its
static transform explicitly:

```bash
ros2 launch wheelchair_bringup wheelchair.launch.py \
  use_navigation:=true \
  publish_base_lidar_tf:=true \
  base_lidar_x:=X base_lidar_y:=Y base_lidar_z:=Z \
  base_lidar_yaw:=YAW base_lidar_pitch:=PITCH base_lidar_roll:=ROLL
```

The six base-LiDAR values default to zero only because ROS launch arguments
require defaults. `publish_base_lidar_tf` defaults to false; never enable it
without supplying the measured values. A future robot-description package may
own this transform instead.

## Scope

This package starts sensor drivers, static sensor transforms, optional local
navigation, and optional RViz. It does not perform calibration, sensor fusion,
Nav2 planning, localization, shared control, CAN communication, or wheelchair
actuation. It starts `rslidar_sdk_node` directly so the vendor package does not
create a second RViz process.

List all launch options with:

```bash
ros2 launch wheelchair_bringup wheelchair.launch.py --show-args
```
