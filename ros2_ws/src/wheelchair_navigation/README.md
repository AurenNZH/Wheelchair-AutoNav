# Wheelchair Navigation

This non-actuating package owns L2 preprocessing and Nav2 costmap production.
It does not publish velocity, select a route, or decide STOP/SLOW/CLEAR.

## Active pipeline

```text
/lidar_right/points
  -> point_support_filter (three points per 0.1 m eligible cell)
  -> /lidar_right/points_filtered
  -> stock Nav2 ObstacleLayer
  -> /nav2_front_costmap
```

The filter transforms each cloud to `base_link`, validates its timestamp,
preserves complete PointCloud2 records, and removes eligible cells containing
fewer than the configured number of points. Finite points outside Nav2's
height/range/front-grid marking window remain for Nav2 to interpret. Successful
filtered publication emits `/lidar_right/filter/source_header`; rejected
low-support points are available on `/lidar_right/low_support_points` when
subscribed.

The Nav2 grid remains 4 m forward by 8 m wide at 0.1 m resolution. Inflation is
disabled by default and must be enabled explicitly for weighted-cost testing.

```bash
ros2 launch wheelchair_navigation nav2_mapping.launch.py use_rviz:=true
ros2 launch wheelchair_navigation nav2_mapping.launch.py \
  use_inflation:=true inflation_radius:=0.55 \
  cost_scaling_factor:=3.0 use_rviz:=true
```

Use `support_min_points_per_cell:=1` only for a pass-through comparison. The
continuity monitor already defaults to the right-L2 topics:

```bash
ros2 run wheelchair_navigation nav2_costmap_monitor
```
