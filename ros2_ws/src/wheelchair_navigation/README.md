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
- Timing and filter counters: `/diagnostics`
- Target frame: `base_link`

`/local_obstacles` retains observed right/side evidence for proximity checks.
`/front_costmap` covers X `0..4 m` and Y `-4..4 m` for requested forward/right
swept-path checks. Both are raw occupied cells with no inflation; wheelchair
geometry is handled by the shared-control supervisor.

The former derived clearance and multipath shadow maps were retired because
they were not safety inputs and the observed ghosting was predominantly an
optical contamination problem. See
`docs/history/airy_multipath_experiment.md`.

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
age, filter time, raw/front occupied cells, chassis-filtered points, rejected
clouds, and lag spikes.

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
