# Filtered Foxy Nav2 Costmap Evaluation

This is a non-actuating evaluation of Foxy Nav2's ObstacleLayer with the
calibrated AIRY artifact filter upstream. It determines whether the filtered
cloud and Nav2 grid remain continuous beyond the legacy mapper's approximately
four-minute failure point.

The experiment publishes `/nav2_front_costmap`, not `/front_costmap`. Its
weighted costs may now feed the safety supervisor during an explicitly
non-actuating shadow run. No freshness bridge, Collision Monitor, planner,
controller, or physical enforcement is involved.

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

Terminal 2 starts the artifact filter, Nav2 costmap, and lifecycle manager.
Inflation remains off for the validated baseline:

```bash
ros2 launch wheelchair_navigation nav2_mapping.launch.py \
  use_inflation:=false use_rviz:=false
```

Confirm Nav2 is active and the isolated map type is correct:

```bash
ros2 lifecycle get /costmap/costmap
ros2 topic info /rslidar_points --verbose
ros2 topic info /rslidar_points_artifact_filtered --verbose
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

The final monitor line must show nonzero raw, filtered, and map counts.
`pass=true` requires a map rate of at least 5.7 Hz, a maximum map gap of
300 ms, at least 90 percent filter publication, and no rejected input clouds.
This is a continuity result, not sensor-to-map latency: the Foxy OccupancyGrid
does not identify which cloud produced it.

## RViz observation

After the headless run, stop terminal 2 and restart it with the dedicated
obstacle-only view:

```bash
ros2 launch wheelchair_navigation nav2_mapping.launch.py use_rviz:=true
```

Inspect these displays:

- `AIRY PointCloud2 (raw)` is the unfiltered sensor input.
- `AIRY PointCloud2 (artifact filtered)` is the Nav2 sensor input.
- `Nav2 Front Costmap (optional inflation)` is the obstacle-only Nav2 grid.

Observe an open scene and place a large soft object at the established clear,
slow, and stop distances. First record whether cells appear and clear. For a
separate shadow decision run, start `shared_control.launch.py` with motion and
geometry decision gates enabled while keeping UDP enforcement disabled; the
supervisor subscribes to `/nav2_front_costmap` by default.

For an inflated A/B run, stop the obstacle-only launch and restart it:

```bash
ros2 launch wheelchair_navigation nav2_mapping.launch.py \
  use_inflation:=true inflation_radius:=0.55 \
  cost_scaling_factor:=3.0 use_rviz:=true
```

The initial gradient is uncalibrated. The supervisor maps costs `1..98` to a
SLOW candidate and `99..100` to a STOP candidate along the requested
trajectory, but those transitions are shadow evidence only until radius,
scaling, and cost thresholds have been physically measured.

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
