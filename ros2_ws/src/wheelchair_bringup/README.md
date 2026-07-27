# Wheelchair Bringup

Top-level ROS2 launch package for repeatable sensor and local-mapping testing. It
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

Mapping remains disabled by default. Start the non-actuating LiDAR-only mapper
with:

```bash
export ROS_LOCALHOST_ONLY=1
ros2 launch wheelchair_bringup wheelchair.launch.py \
  use_camera:=false use_mapping:=true use_rviz:=true
```

`ROS_LOCALHOST_ONLY=1` avoids ROS 2 Foxy discovery problems observed on the
Jetson while Ethernet is connected to the LiDAR and Wi-Fi is also active. Use
it when the driver, mapper, and RViz all run on the Jetson. Leave it unset when
RViz or another ROS node must run on a different computer.

The measured translations and yaw values are defaults. Pitch and roll are
temporarily zero and all six values per sensor remain launch arguments for
calibration overrides. The RealSense driver owns `camera_link`'s optical-frame
children; do not publish a second optical transform.

The mapper publishes raw returns on `/local_obstacles` and a separately derived
`/local_costmap`. Inflation is zero by default until the chair footprint is
measured. The initial filters accept points from `0.30` to `4.00` metres and
from `0.05` to `1.50` metres above the `base_link` ground plane.

## Scope

This package starts sensor drivers, static sensor transforms, optional local
mapping, and optional RViz. It does not perform calibration, sensor fusion,
Nav2 planning, localization, shared control, CAN communication, or wheelchair
actuation. It starts `rslidar_sdk_node` directly so the vendor package does not
create a second RViz process.

List all launch options with:

```bash
ros2 launch wheelchair_bringup wheelchair.launch.py --show-args
```
