# Stock Foxy Nav2 Costmap Evaluation

This is a non-actuating evaluation of Foxy Nav2's unmodified ObstacleLayer.
It determines whether Nav2 continues receiving the AIRY cloud and publishing
a raw obstacle grid beyond the legacy mapper's approximately four-minute
failure point.

The experiment publishes `/nav2_front_costmap`, not `/front_costmap`. Do not
start the safety supervisor or the Pi enforcement program during this stage.
No project artifact filtering, minimum point threshold, freshness bridge,
Collision Monitor, planner, or controller is involved.

## One-time installation and build

The Jetson user must run the package installation because `sudo` may request
the local password:

```bash
sudo apt-get update
sudo apt-get install ros-foxy-nav2-costmap-2d \
  ros-foxy-nav2-lifecycle-manager
```

Then build:

```bash
cd /home/jetson-xavier-wheelchair/Wheelchair-AutoNav/ros2_ws
source /opt/ros/foxy/setup.bash
colcon build --symlink-install --packages-select \
  wheelchair_navigation wheelchair_bringup
source install/setup.bash
```

Use the following setup at the beginning of every terminal:

```bash
cd /home/jetson-xavier-wheelchair/Wheelchair-AutoNav/ros2_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash
export ROS_LOCALHOST_ONLY=1
```

## Headless six-minute run

Terminal 1 starts the AIRY and its measured static transform, without the
legacy mapper or RViz:

```bash
ros2 launch wheelchair_bringup wheelchair.launch.py \
  use_lidar:=true use_camera:=false use_mapping:=false use_rviz:=false
```

Terminal 2 starts only the stock Nav2 costmap and lifecycle manager:

```bash
ros2 launch wheelchair_navigation nav2_mapping.launch.py use_rviz:=false
```

Confirm Nav2 is active and the isolated map type is correct:

```bash
ros2 lifecycle get /costmap/costmap
ros2 topic info /rslidar_points --verbose
ros2 topic info /nav2_front_costmap --verbose
```

The lifecycle state must be `active`; `/nav2_front_costmap` must be
`nav_msgs/msg/OccupancyGrid`. Foxy's stock PointCloud2 subscription is best
effort with depth 50. That is intentionally left unchanged for this test.

Terminal 3 records the continuity result. It ignores ten warm-up seconds and
then exits automatically after six minutes:

```bash
ros2 run wheelchair_navigation nav2_costmap_monitor
```

Optionally record Jetson utilization in terminal 4 and stop it after the
monitor exits:

```bash
tegrastats --interval 1000 --logfile /tmp/nav2_costmap_tegrastats.txt
```

The final monitor line must show nonzero cloud and map counts. `pass=true`
means the cloud and map stayed at least 9 Hz and neither arrival stream had a
gap above 300 ms. This is a continuity result, not sensor-to-map latency: the
stock Foxy OccupancyGrid does not identify which cloud produced it.

## RViz observation

After the headless run, stop terminal 2 and restart it with the dedicated raw
view:

```bash
ros2 launch wheelchair_navigation nav2_mapping.launch.py use_rviz:=true
```

Inspect these displays:

- `AIRY PointCloud2 (raw)` is the unfiltered sensor input.
- `Stock Nav2 ObstacleLayer` is the unfiltered Nav2 grid.

Observe an open scene and place a large soft object at the established clear,
slow, and stop distances. At this stage record whether cells appear and clear;
do not expect supervisor decisions because `/front_costmap` remains
disconnected.

## Interpretation and next decision

- If `/rslidar_points` and `/nav2_front_costmap` both stop, investigate the
  driver or DDS path before changing costmap code.
- If clouds continue but the Nav2 map stops, capture the lifecycle state,
  terminal-2 logs, topic endpoint details, and tegrastats output.
- If Nav2 stays live but artifacts remain, first compare its built-in height,
  range, footprint clearing, and raytracing behaviour. The VoxelLayer is an
  optional later A/B test.
- Only after this run should a source-timestamp bridge or custom artifact layer
  be considered. Until then, keep the supervisor and physical enforcement off.
