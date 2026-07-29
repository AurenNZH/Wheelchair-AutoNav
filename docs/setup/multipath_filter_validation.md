# AIRY Multipath Shadow-Filter Validation

This procedure evaluates scattered cells observed when AIRY is within roughly
1.4 m of an obstacle. It does not authorize autonomous or powered movement.
Keep drive power isolated and leave every motion-enable setting false.

## 1. Clean launch

Stop every earlier ROS launch terminal. Start one camera-free mapping session:

```bash
cd /home/jetson-xavier-wheelchair/Wheelchair-AutoNav
source /opt/ros/foxy/setup.bash
source ros2_ws/install/setup.bash
export ROS_LOCALHOST_ONLY=1
ros2 launch wheelchair_bringup wheelchair.launch.py \
  use_lidar:=true use_camera:=false use_mapping:=true use_rviz:=true
```

In a second terminal, use the compact monitor:

```bash
cd /home/jetson-xavier-wheelchair/Wheelchair-AutoNav
source /opt/ros/foxy/setup.bash
source ros2_ws/install/setup.bash
export ROS_LOCALHOST_ONLY=1
ros2 run wheelchair_navigation mapping_monitor
```

Do not collect evidence until it reports fresh diagnostics near the AIRY frame
rate and no duplicate `/rslidar_sdk_node`.

## 2. Understand the three maps

Toggle one RViz map at a time:

- `Front 180 Costmap (Raw)` is unchanged sensor evidence.
- `Front 180 Costmap (Shadow Filtered)` keeps multi-cell or persistent cells.
- `Front Cells Rejected as Flicker` shows what only the shadow map removed.

A blank cell in the filtered map is not confirmed free. The raw map remains
the source of immediate evidence throughout this validation.

## 3. Fixed captures

Keep the wheelchair stationary. Capture 30 seconds for an open corridor and
for each test target at 2.0, 1.4, 1.0, and 0.6 m:

- flat wall;
- broom;
- stationary and moving human leg;
- complete stationary and walking human;
- table and chairs.

Start each recording from the repository root, choosing a unique output name:

```bash
timeout --signal=INT 30 ros2 bag record \
  -o /tmp/airy_multipath_SCENE_DISTANCE \
  /rslidar_points /tf /tf_static /local_obstacles /front_costmap \
  /front_costmap_filtered /front_costmap_rejected /diagnostics
```

For every capture, note the target, measured distance, material, whether ghost
cells flickered or persisted, and whether any real target cell appeared in the
rejected layer.

## 4. Pass criteria

The shadow filter passes only if:

- at least 90% of annotated close-obstacle ghost cells move to the rejected
  layer;
- every broom, leg, person, table, and chair is present in at least 99% of
  frames after the first two scans;
- no real target disappears for more than two consecutive frames;
- `ghost_filter_ms` p95 is below 5 ms;
- total processing p95 is below 100 ms and maximum below 150 ms;
- the footrest exclusion box and both raw maps behave exactly as before.

Then run all scenarios continuously for ten minutes and require zero repeating
latency spikes or growing backlog.

## 5. Failure path

If a real small object is lost, keep the shadow map isolated and reduce
filtering strength; never promote it to navigation. If the spatial-temporal
filter removes less than 90% of ghosts, compare AIRY intensity distributions
from labelled real and ghost returns. Add an intensity condition only if it
retains every labelled real return while rejecting at least 80% of the
remaining ghosts.

If neither method passes, do not broaden software rejection. Treat the close
reflective condition as unknown/unsafe and continue with a non-reflective hood
or independent LiDAR coverage.
