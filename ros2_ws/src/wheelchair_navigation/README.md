# Wheelchair Navigation

This non-actuating package owns L2 preprocessing and Nav2 costmap production.
It does not publish velocity, select a route, or decide STOP/SLOW/CLEAR.

## Active pipeline

```text
/lidar_right/points -> right point_support_filter -+
                                                   +-> stock Nav2 ObstacleLayer
/lidar_left/points  -> left point_support_filter  -+-> /nav2_merged_costmap
```

Each filter transforms its cloud to `base_link`, validates its timestamp,
applies its independently calibrated chassis-artifact boxes, and removes cells
with insufficient support. The global threshold remains three points; cells
inside each box's 0.10 m XY halo require fifteen points. Filtered output is a
compact XYZ cloud in `base_link`, preserving the source acquisition stamp, and
finite points outside Nav2's marking window remain for Nav2 to interpret. Each
successful filtered publication emits the matching
`/lidar_<side>/filter/source_header`; rejected low-support points are available
on `/lidar_<side>/low_support_points`, while hard-rejected points are available
on `/lidar_<side>/artifact_rejected_points` when subscribed.

The right-L2 rule includes a separate elevated JSM box at
`x=0.50..0.60`, `y=-0.40..-0.20`, `z=0.77..0.87` to remove the measured
physical-JSM return without extending the full chassis box upward.

The Nav2 grid spans `x=-0.6..4.4 m` and `y=-4.0..4.0 m` at 0.1 m resolution.
The same origin-aware grid makes the three-point support rule apply behind
`base_link`. Inflation is disabled by default and must be enabled explicitly
for weighted-cost testing.

```bash
ros2 launch wheelchair_navigation nav2_mapping.launch.py use_rviz:=true
ros2 launch wheelchair_navigation nav2_mapping.launch.py \
  use_inflation:=true inflation_radius:=0.45 \
  cost_scaling_factor:=3.0 use_rviz:=true
```

Use `support_min_points_per_cell:=1` only for a pass-through comparison. The
artifact rules default on and can be isolated with
`use_right_artifact_filter:=false` or `use_left_artifact_filter:=false`. Their
empty-scene calibration does not replace real-obstacle preservation testing.
The continuity monitor still defaults to the right-L2 topics. Dual-source
shared-control freshness remains a separate validation step:

```bash
ros2 run wheelchair_navigation nav2_costmap_monitor
```
