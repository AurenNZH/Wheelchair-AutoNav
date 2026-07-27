# Wheelchair Local Mapping

ROS2 package for the non-actuating RoboSense AIRY local-mapping baseline. It
does not publish velocity commands or trajectories and never controls the
wheelchair.

## Geometry

`base_link` is the ground projection of the wheelchair centre/rear-axle
midpoint, with X forward, Y left, and Z up. The currently measured AIRY mount
is:

```text
base_link -> rslidar: -0.265 -0.330 0.320 -0.78540 0 0
```

The zero pitch and roll values are provisional. Calibrate them before treating
the map as collision evidence.

## Build and run

```bash
cd ~/Wheelchair-AutoNav/ros2_ws
source /opt/ros/foxy/setup.bash
colcon build --symlink-install --packages-select wheelchair_navigation
source install/setup.bash
ros2 launch wheelchair_navigation local_mapping.launch.py
```

Interfaces:

- Input: `/rslidar_points` (`sensor_msgs/PointCloud2`, best effort, depth 1)
- Raw obstacle cells: `/local_obstacles`
- Derived clearance grid: `/local_costmap`
- Timing and filter counters: `/diagnostics`
- Target frame: `base_link`

`/local_obstacles` contains rasterized measurements after range, height, and
measured self-body filtering. `/local_costmap` is a separate copy on which
optional clearance inflation is applied once. `inflation_radius_m` defaults to
zero so clearance padding cannot be mistaken for obstacle size during sensor
validation.

The mapper rejects invalid, stale, future-dated, empty, or untransformable
clouds and reports the reason through diagnostics. It does not publish a stop
command because this milestone has no motion-command interface.

## Chair and footrest measurement

Do not estimate self-filter boxes from the LiDAR position. Measure them:

1. Mark the `base_link` origin on the floor at the centre/rear-axle projection.
2. Remeasure LiDAR X, Y, Z and yaw; estimate pitch and roll from level floor and
   wall point planes.
3. Measure the chair body, wheel/caster swing envelope, and each footrest as
   `[min_x, max_x, min_y, max_y, min_z, max_z]` in `base_link`.
4. Repeat for every supported footrest adjustment and use the union envelope.
5. Put the groups into `config/local_mapping.yaml`; the default 0.02 m padding
   is applied on all faces.
6. Verify in RViz that known targets immediately outside every box remain in
   `/local_obstacles`.

## Reflection capture protocol

Use the same chair pose and environment for each short capture:

1. Empty scene with bare footrests.
2. Empty scene with a removable matte covering on the footrests.
3. A measured box inside the affected viewing sector.
4. The same box immediately to either side of that sector.

Record `/rslidar_points`, `/tf`, and `/tf_static`. For driver-level replay,
temporarily enable `send_packet_ros` in the AIRY configuration and also record
`/rslidar_packets`. Store large bags under `experiments/` or external storage,
not in Git. Compare intensity, ring, elevation, and persistence before adding
any live reflection threshold.

Physical treatment is accepted when it removes at least 95% of annotated ghost
cells without losing known targets. If it does not, the affected ray sector
must be treated as unknown rather than assumed clear; autonomous movement
remains out of scope until independent sensing covers it.

## Performance acceptance

On the Jetson, representative 86k- and 172k-point clouds must take less than
100 ms to process. At 10 Hz, the 95th-percentile source-stamp-to-map age must be
below 150 ms without a growing message backlog. Inspect live values with:

```bash
ros2 topic echo /diagnostics
ros2 topic hz /local_obstacles
```

## Test

```bash
colcon test --packages-select wheelchair_navigation
colcon test-result --verbose
```
