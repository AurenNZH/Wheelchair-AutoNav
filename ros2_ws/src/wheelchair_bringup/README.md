# Wheelchair Bringup

Top-level ROS2 launch package for repeatable sensor and navigation testing. It
does not command the wheelchair.

## Build and Source

The AIRY driver and packet messages are pinned as Git submodules in this
workspace. Initialize them after cloning the repository:

```bash
git submodule update --init --recursive
source /opt/ros/foxy/setup.bash
cd ~/Wheelchair-AutoNav/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

The project-owned `config/rslidar_airy.yaml` preserves the RSAIRY settings
without modifying the vendor submodule.

## Launch

Start both sensors and publish their measured transforms beneath `base_link`:

```bash
ros2 launch wheelchair_bringup wheelchair.launch.py
```

Add the installed RViz view:

```bash
ros2 launch wheelchair_bringup wheelchair.launch.py use_rviz:=true
```

Navigation remains disabled by default. Start the current non-actuating,
LiDAR-only navigation prototype with:

```bash
ros2 launch wheelchair_bringup wheelchair.launch.py \
  use_navigation:=true use_rviz:=true
```

The measured translations and yaw values are defaults. Pitch and roll are
temporarily zero and all six values per sensor remain launch arguments for
calibration overrides. The RealSense driver owns `camera_link`'s optical-frame
children; do not publish a second optical transform.

The initial local costmap accepts points from `0.30` to `4.00` metres and from
`0.05` to `1.50` metres above the `base_link` ground plane.

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
