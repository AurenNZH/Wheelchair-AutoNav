# Wheelchair Bringup

Top-level sensor, calibrated-TF, mapping, and RViz launch package. It does not
command the wheelchair.

The default profile is LiDAR-first: AIRY is enabled, RealSense is disabled,
mapping is disabled, and RViz is disabled. Start the mapping view with:

```bash
cd /home/jetson-xavier-wheelchair/Wheelchair-AutoNav
source /opt/ros/foxy/setup.bash
source ros2_ws/install/setup.bash
export ROS_LOCALHOST_ONLY=1
ros2 launch wheelchair_bringup wheelchair.launch.py \
  use_lidar:=true use_camera:=false use_mapping:=true use_rviz:=true \
  runtime_profile:=safety
```

`ROS_LOCALHOST_ONLY=1` avoids Foxy discovery problems when the Jetson uses
Ethernet for AIRY and Wi-Fi for the router. Leave it unset when another
computer must join the ROS graph.

The default `safety` runtime profile publishes `/front_costmap` first, skips
the unused surround/artifact products, and opens a lightweight 10 FPS map
view. Use `runtime_profile:=artifact_debug` only when calibrating the full AIRY
point, filtered-map, rejection, and marker displays; that profile is not for
physical enforcement.

The measured AIRY transform defaults to:

```text
base_link -> rslidar: 0.330 -0.265 0.320 1.04720 0 0
```

All transform values remain launch arguments for controlled calibration.
List every option with:

```bash
ros2 launch wheelchair_bringup wheelchair.launch.py --show-args
```
