# Nav2 Costmap Evaluation

The production mapping path uses the right Unitree L2, a three-point support
filter, and Foxy's unmodified Nav2 `ObstacleLayer`. It publishes
`/nav2_front_costmap` and never commands motion.

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

## Interface checks

```bash
ros2 topic info /lidar_right/points --verbose
ros2 topic hz /lidar_right/points
ros2 topic hz /lidar_right/points_filtered
ros2 topic hz /lidar_right/filter/source_header
ros2 topic hz /nav2_front_costmap
ros2 run wheelchair_navigation nav2_costmap_monitor
```

The raw and filtered clouds must advance at comparable rates, the heartbeat
must follow successful filtered publication, and the Nav2 map must remain
continuous without a growing queue. Stopping the filter must stop both the
filtered cloud and heartbeat. Stopping Nav2 must stop the map while leaving the
filter streams alive.

Validate measured obstacle distances, thin poles, low blocks, empty scenes,
doorways, and each edge of the supported forward/right envelope. The right-only
system must not depict unobserved left, rear, step, curb, or drop-off regions as
safe. Record the exact launch arguments and sensor mounting state with every
acceptance run.
