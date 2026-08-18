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
- Low-support points removed from the shadow: `/artifact_filter/low_support_points`
- Configured staircase regions and cell-aligned halo outlines:
  `/artifact_filter/masks`
- Exact grid cells evaluated by thresholding: `/artifact_filter/threshold_cells`
- Timing and filter counters: `/diagnostics`
- Target frame: `base_link`

`/local_obstacles` retains observed right/side evidence for proximity checks.
`/front_costmap` covers X `0..4 m` and Y `-4..4 m` for requested forward/right
swept-path checks. Both are raw occupied cells with no inflation; wheelchair
geometry is handled by the shared-control supervisor.

The artifact-filtered map is explicitly a **SHADOW ONLY** diagnostic. Neither
shared control nor the safety supervisor consumes it. A mask error or an
`artifact_filter_frame` that differs from the target frame suppresses shadow
products, reports an error on `/diagnostics`, and leaves both raw maps
operational.

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

## Stock Nav2 Evaluation

`nav2_mapping.launch.py` is an isolated evaluation of Foxy Nav2's unmodified
`ObstacleLayer`, fed by the calibrated artifact-filtered PointCloud2 by
default. It publishes `/nav2_front_costmap`; it does not publish
`/front_costmap`, launch the safety supervisor, or command the wheelchair.
An optional Nav2 `InflationLayer` is disabled by default and remains a
non-authoritative RViz/evaluation feature.

Install the Foxy runtime once:

```bash
sudo apt-get install ros-foxy-nav2-costmap-2d \
  ros-foxy-nav2-lifecycle-manager
```

Build this package, start the AIRY driver without the legacy mapper, and then
launch the stock Nav2 profile:

```bash
ros2 launch wheelchair_navigation nav2_mapping.launch.py use_rviz:=false
```

Enable the uncalibrated weighted-cost A/B profile explicitly:

```bash
ros2 launch wheelchair_navigation nav2_mapping.launch.py \
  use_inflation:=true inflation_radius:=0.55 \
  cost_scaling_factor:=3.0 use_rviz:=true
```

Both modes publish `/nav2_front_costmap`; restart the launch to switch. The
inflated costs do not yet encode the supervisor's CLEAR, SLOW, or STOP policy.

Measure both source and map continuity for six minutes:

```bash
ros2 run wheelchair_navigation nav2_costmap_monitor
```

The monitor deliberately does not report Nav2's OccupancyGrid header as sensor
latency: Foxy does not preserve the cloud acquisition timestamp there. A
`pass=true` result means only that both streams maintained the configured rate
and avoided gaps over 300 ms. See
`docs/setup/nav2_costmap_evaluation.md` for the complete procedure.

Monitor performance:

```bash
ros2 run wheelchair_navigation mapping_monitor
```

The monitor reports effective rate, current/p95/maximum processing time, cloud
age/p95, point and artifact-filter time, raw/shadow front cells,
chassis-filtered and artifact-rejected points, rejected clouds, and lag spikes.

## Artifact Shadow Calibration

`artifact_grid_mask_cells` contains flat groups in `base_link`:

```text
[region_id, forward_cell, lateral_cell, min_z_m, max_z_m]
```

Each XY index names one exact 10 cm front-costmap cell. Forward index zero is
X `0.0..0.1 m`; lateral index zero is Y `0.0..0.1 m`, with negative values to
the wheelchair's right. The checked-in 76 records form three provisional
regions with per-cell Z bands. RViz renders three consolidated translucent
staircase meshes and their exterior outlines without labels. These values are
not approved safety geometry. Calibrate them from recorded `/rslidar_points`
bags by following `docs/setup/airy_artifact_shadow_validation.md`.

After cell-band rejection, the shadow counts remaining points in each front
cell. Every cell requires `artifact_global_min_points_per_cell` points; its
current value is three. `artifact_grid_halo_spans` defines a stricter local
scope independently as flat groups:

```text
[region_id, forward_cell, min_lateral_cell, max_lateral_cell]
```

Both lateral bounds are inclusive. Multiple spans can create an arbitrary
staircase shape; every mask cell must remain covered by its region's halo. The
checked-in spans reproduce the former one-cell, eight-neighbour dilation, but
can now be expanded or reshaped without changing any mask record. Configured
halo cells require at least `artifact_min_points_per_cell` unmasked points;
the current value is fifteen. Set both thresholds to one at runtime to compare
against the geometry-only shadow:

```bash
ros2 param set /local_costmap artifact_global_min_points_per_cell 1
ros2 param set /local_costmap artifact_min_points_per_cell 1
ros2 param set /local_costmap artifact_global_min_points_per_cell 3
ros2 param set /local_costmap artifact_min_points_per_cell 15
```

Cell-band-rejected points are magenta in RViz; low-support points are yellow.
Bright green stepped outlines show the exact configured halo footprints. The
threshold scope paints those cells cyan and overlays cells that fail the
current support rule in yellow. The raw Front 180 display defaults off so it
cannot show through cells removed from the shadow, but remains available as a
comparison checkbox.

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
