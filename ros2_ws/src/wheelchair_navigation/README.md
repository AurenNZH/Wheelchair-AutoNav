# Wheelchair Local Mapping

ROS2 package for the non-actuating RoboSense AIRY local-mapping baseline. It
does not publish velocity commands or trajectories and never controls the
wheelchair.

## Geometry

`base_link` is the ground projection of the wheelchair centre/rear-axle
midpoint, with X forward, Y left, and Z up. The currently measured AIRY mount
is:

```text
base_link -> rslidar: 0.330 -0.265 0.320 1.04720 0 0
```

The measured pitch and roll are both `0.0`. The pose was converted from the
legacy project axes (X left, Y backward) to the ROS mobile-base convention
(X forward, Y left) using `x_new=-y_old`, `y_new=x_old`, and
`yaw_new=yaw_old+pi/2`. That conversion produced the initial `+0.78540`
estimate; a physical forward-target test refined and validated the final
AIRY yaw as `+1.04720` rad (60 degrees). A target directly ahead of the AIRY
now appears ahead of its translated origin and to the right of `base_link`.

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
- Optional derived clearance grid: `/local_costmap`
- Raw front 180-degree grid: `/front_costmap`
- Shadow-filtered front grid: `/front_costmap_filtered`
- Cells rejected only from the shadow grid: `/front_costmap_rejected`
- Timing and filter counters: `/diagnostics`
- Target frame: `base_link`

`/local_obstacles` contains rasterized observed measurements after range,
height, and measured self-body filtering. It must not be interpreted as proof
of full 360-degree coverage: the chair body occludes AIRY returns on the left.
`/local_costmap` is an optional inflated copy and is disabled in the default
demo profile because inflation is zero. `/front_costmap` covers base-link
X = 0 to 4 m and Y = -4 to 4 m. Front always means wheelchair-forward
(`base_link +X`), never an untransformed LiDAR axis.

The shadow filter never changes `/local_obstacles` or `/front_costmap`. A
component occupying at least two connected 0.1 m cells is retained
immediately. An isolated cell is retained only after it appears within one
cell in at least two of three frames. This gives flickering multipath a
separate evaluation layer without allowing the filtered result to influence
navigation or actuation. Toggle the three front-map displays individually in
RViz when comparing them.

The mapper rejects invalid, stale, future-dated, empty, or untransformable
clouds and reports the reason through diagnostics. It does not publish a stop
command because this milestone has no motion-command interface.

The diagnostics separate decoding, TF, filtering, raw rasterization, optional
inflation, front selection/rasterization, and each publication. A rolling
120-sample window reports processing p50, p95, maximum, and counts above the
150 ms spike threshold. Source/arrival period, cloud age, and point/cell
counters distinguish driver/network delay from mapping work.

Use the compact monitor instead of piping the full diagnostic message:

```bash
ros2 run wheelchair_navigation mapping_monitor
```

It prints one line per second and explicitly identifies missing diagnostics
and duplicate ROS node names.

## Target-axis check

Do not rotate the front grid to compensate for an apparent sensor-axis error.
Correct the TF so every consumer sees a consistent robot frame:

1. Mark targets at `(2.0, 0.0)`, `(0.0, 2.0)`, and `(0.0, -2.0)` metres in
   `base_link`.
2. Record `/rslidar_points`, `/tf`, and `/tf_static`.
3. Adjust only `base_lidar_yaw` for the driver-native axis convention; retain
   the measured translation and confirmed zero pitch/roll.
4. Accept the result only when the forward target has positive X and
   `abs(Y) <= 0.1 m`, the left target has positive Y, and the right target has
   negative Y.
5. Record the accepted yaw and the bag used to derive it.

## Chair body and mount measurement

The measured chair body is 1.00 m long and 0.53 m wide, centred at
`base_link`. This is not yet the collision footprint: wheels, casters,
footrests, mounts, and their swept positions must be added before
`geometry_calibrated` can become true.

Do not mark the body permanently occupied in a sensor obstacle map. That would
make the supervisor collide with itself and would not remove multipath returns
reported outside the body. Keep robot geometry and sensor observations
separate.

The confirmed reflection source is the footrest mounts, approximately 4 cm
above the AIRY. Fuzzy hook-and-loop covering produced only a mediocre
reduction, so a non-reflective hood remains a future experiment. The current
provisional exclusion volume was inspected in RViz and is:

```text
[min_x, max_x, min_y, max_y, min_z, max_z]
[0.00, 0.53, -0.465, 0.20, 0.32, 0.82]
```

It uses zero padding, so those are also the effective filter bounds. It must
still pass the near-boundary target and latency checks before it is accepted.
For any replacement or additional boxes:

1. Mark the `base_link` origin on the floor at the centre/rear-axle projection.
2. Remeasure LiDAR X, Y, Z and yaw; verify the confirmed zero pitch and roll
   against level-floor and wall planes.
3. Measure each mount, wheel/caster swing envelope, and each footrest as
   `[min_x, max_x, min_y, max_y, min_z, max_z]` in `base_link`.
4. Repeat for every supported footrest adjustment and use the union envelope.
5. Put the groups into `config/local_mapping.yaml`; keep padding explicit and
   account for it when evaluating the effective exclusion volume.
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

The AIRY was empirically reliable to 5 m, but the demo decision map remains
4 m to preserve latency. The current single-sensor scope is level, inspected
indoor floors and forward/right motion only. Low obstacles, holes, steps,
curbs, drop-offs, reverse, and left turns are unsupported.

## Performance acceptance

On the Jetson, a ten-minute static and moving-obstacle run must have processing
p95 below 100 ms, processing maximum below 150 ms, cloud-age p95 below 150 ms,
ghost-filter p95 below 5 ms, and no repeating lag spikes or growing message
backlog. Inspect live values with:

```bash
ros2 run wheelchair_navigation mapping_monitor
ros2 topic hz /local_obstacles
```

Follow `docs/setup/multipath_filter_validation.md` before considering the
shadow-filtered map for any downstream use.

## Test

```bash
colcon test --packages-select wheelchair_navigation
colcon test-result --verbose
```
