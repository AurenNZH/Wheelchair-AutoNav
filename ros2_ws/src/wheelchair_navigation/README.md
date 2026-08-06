# Wheelchair Local Mapping

Non-actuating RoboSense AIRY obstacle mapping. This package never publishes
velocity, trajectories, or CAN commands.

## Geometry

`base_link` uses X forward, Y left, and Z up. The physically validated mount is:

```text
base_link -> rslidar: 0.330 -0.265 0.320 1.04720 0 0
```

AIRY pitch and roll are zero. The 60-degree yaw was validated with measured
targets after converting the legacy project axes to the ROS mobile-base
convention.

## Interfaces

- Input: `/rslidar_points` (`sensor_msgs/PointCloud2`, best effort, depth 1)
- Full local raw obstacles: `/local_obstacles`
- Robot-forward 180-degree raw obstacles: `/front_costmap`
- Diagnostic artifact shadow: `/front_costmap_artifact_filtered`
- Sensor-frame points removed from the shadow: `/artifact_filter/rejected_points`
- Low-support points removed from halo cells: `/artifact_filter/low_support_points`
- Configured mask boxes: `/artifact_filter/masks`
- Timing and filter counters: `/diagnostics`
- Target frame: `base_link`

`/local_obstacles` retains observed right/side evidence for proximity checks.
`/front_costmap` covers X `0..4 m` and Y `-4..4 m` for requested forward/right
swept-path checks. Both are raw occupied cells with no inflation; wheelchair
geometry is handled by the shared-control supervisor.

The artifact-filtered map is explicitly a **SHADOW ONLY** diagnostic. Neither
shared control nor the safety supervisor consumes it. A mask error or a cloud
whose native frame is not `artifact_filter_frame` suppresses shadow products,
reports an error on `/diagnostics`, and leaves both raw maps operational.

The former temporal 2D multipath filter remains retired. The current filter is
a narrower 3D experiment for repeatable, flat near-sensor traces. See
`docs/history/2026-08-05_airy_flat_artifact_filter_decision.md`.

## Build and Run

```bash
cd /home/jetson-xavier-wheelchair/Wheelchair-AutoNav
source /opt/ros/foxy/setup.bash
cd ros2_ws
colcon build --symlink-install --packages-select wheelchair_navigation
source install/setup.bash
ros2 launch wheelchair_navigation local_mapping.launch.py
```

Monitor performance:

```bash
ros2 run wheelchair_navigation mapping_monitor
```

The monitor reports effective rate, current/p95/maximum processing time, cloud
age/p95, point and artifact-filter time, raw/shadow front cells,
chassis-filtered and artifact-rejected points, rejected clouds, and lag spikes.

## Artifact Shadow Calibration

`artifact_pancake_masks` contains flat groups in native `rslidar` coordinates:

```text
[start_x, start_y, end_x, end_y, half_width, min_z, max_z]
```

Each group is an oriented rectangular prism centred on the XY segment, with a
fixed Z band. The checked-in list contains three provisional visual-debug
prisms measured from a live sensor-frame cloud on 2026-08-06. RViz renders each
as a translucent box, bright outline, and label. These values demonstrate and
tune the filter; they are not approved safety geometry. Calibrate them from
recorded `/rslidar_points` bags by following
`docs/setup/airy_artifact_shadow_validation.md`. Do not widen a volume just to
make an overlay look clean.

After prism rejection, the shadow counts remaining points in each 10 cm front
cell. Within `artifact_threshold_halo_m` of a prism, a cell must contain at
least `artifact_min_points_per_cell` unmasked points. The provisional defaults
are 0.10 m and two points. The rule is local: cells outside the halo retain
one-point sensitivity. Set the minimum to one at runtime to compare against
the geometry-only shadow:

```bash
ros2 param set /local_costmap artifact_min_points_per_cell 1
ros2 param set /local_costmap artifact_min_points_per_cell 2
```

Prism-rejected points are magenta in RViz; low-support points are yellow. The
raw Front 180 display defaults off so it cannot show through cells removed from
the shadow, but remains available as a comparison checkbox.

## Chassis Reflections

The current measured exclusion volume in `base_link` is:

```text
[min_x, max_x, min_y, max_y, min_z, max_z]
[0.00, 0.53, -0.465, 0.20, 0.32, 0.82]
```

Padding is zero. Keep this filter enabled while developing the non-reflective
hood. Before every capture:

1. inspect and clean the AIRY dome with a sensor-safe method;
2. confirm the hood is secure and does not block the forward/right envelope;
3. place measured targets immediately outside each box face;
4. confirm every target remains in both raw maps.

Compare the hood with the self-filter enabled and disabled. Remove or shrink
the digital box only after the hood eliminates chassis returns and every
boundary target remains visible.

## Coverage and Performance Gates

The single AIRY is accepted only for level, inspected indoor forward/right
operation. Chair occlusion makes left turns unsupported. Reverse, stairs,
curbs, holes, drop-offs, and low hazards beneath the observed height are not
covered.

Over ten minutes at the AIRY frame rate:

- processing p95 must remain below 100 ms;
- processing maximum and cloud-age p95 must remain below 150 ms;
- no repeating latency spike or growing backlog may occur;
- obstacle dimensions and positions must remain credible.

## Tests

```bash
colcon test --packages-select wheelchair_navigation
colcon test-result --verbose
```
