# Nav2 Costmap Evaluation

The production mapping path uses both Unitree L2 sensors, one identical
three-point support filter per sensor, and Foxy's unmodified Nav2
`ObstacleLayer` with two observation sources. It publishes
`/nav2_merged_costmap` and never commands motion. The robot-relative grid spans
`x=-0.6..4.4 m` and `y=-4.0..4.0 m` at 0.1 m resolution.

## Launch

```bash
source /opt/ros/foxy/setup.bash
source ros2_ws/install/setup.bash
ros2 launch wheelchair_bringup wheelchair.launch.py \
  use_lidar:=true use_mapping:=true use_rviz:=true
```

For a filter comparison, set `support_min_points_per_cell:=1` in a separate
navigation-only launch. For weighted safety-policy evaluation, explicitly set
`use_inflation:=true`; inflation remains disabled by default.

The independently calibrated L2 hard boxes and fifteen-point halos are enabled
by default. Isolate either rule with `use_right_artifact_filter:=false` or
`use_left_artifact_filter:=false` while leaving the global support rule active.

## Interface checks

```bash
ros2 topic info /lidar_right/points --verbose
ros2 topic hz /lidar_right/points
ros2 topic hz /lidar_right/points_filtered
ros2 topic hz /lidar_right/filter/source_header
ros2 topic hz /lidar_left/points
ros2 topic hz /lidar_left/points_filtered
ros2 topic hz /lidar_left/filter/source_header
ros2 topic hz /nav2_merged_costmap
ros2 run wheelchair_navigation nav2_costmap_monitor
```

RViz exposes each static hard box and halo. The optional rejected-point topics
are `/lidar_<side>/artifact_rejected_points`; enable them only while diagnosing
or calibrating to avoid unnecessary debug-cloud construction.

The raw and filtered clouds must advance at comparable rates, the heartbeat
must follow successful filtered publication, and the Nav2 map must remain
continuous without a growing queue. Stopping the filter must stop both the
filtered cloud and heartbeat. Stopping Nav2 must stop the map while leaving the
filter streams alive.

Validate measured obstacle distances, thin poles, low blocks, empty scenes,
doorways, the overlap region, the `x=-0.6 m` rear boundary, and each lateral
edge of the mapped envelope. Test right-only, left-only, and dual-source
behavior before accepting the merged map. Rear map coverage does not enable
reverse intervention. Hard turns use only the 0.55 m base-centred pixelated
disc and require both filter heartbeats. The system must not depict unobserved
step, curb, or drop-off regions as safe. Record the exact launch arguments and
sensor mounting state with every acceptance run.
