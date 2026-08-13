# AIRY Artifact Shadow Calibration and Validation

This procedure calibrates and evaluates diagnostic-only staircase masks. The
raw `/front_costmap` remains the safety supervisor input throughout.

## Recordings

For each case, record 10–20 seconds while stationary and preserve the original
cloud frame and timestamps:

```bash
ros2 bag record -o ~/wheelchair_bags/clear_lidar_fix_03 \
  /rslidar_points /tf /tf_static /front_costmap \
  /front_costmap_artifact_filtered /local_obstacles /diagnostics
```

Capture open floor, black acrylic, and the installed shielded wheelchair. Also
capture a wide panel, vertical pole, low block, and thin horizontal bar crossing
every prospective mask at stop, slow, and clear distances. Existing
occupancy-grid-only bags cannot be used to recover sensor-frame height. Retain a
raw AIRY PCAP when practical for a non-blocking RoboSense support report.

Name bags and evidence with UTC timestamps and record target dimensions,
positions, expected STOP/SLOW/CLEAR label, AIRY/hood condition, software commit,
and mask configuration alongside each run.

## Deriving one staircase region

Do not use PCA or clustering.

1. Transform the labelled trace into `base_link` with the recorded TF and map
   each point to its exact front-costmap cell.
2. Select a tight contiguous staircase of cells around returns that persist in
   at least 10% of frames. Do not fill the broad convex hull between traces.
3. For every selected cell, set the Z interval to the smallest band containing
   at least 99% of its labelled trace. A filled staircase cell with no samples
   inherits the nearest source cell's band; tied bands are unioned.
4. Keep region IDs contiguous from zero. Cross-region cell overlap is allowed
   and is union-counted; duplicate cells within one region are invalid.
5. Enter flat groups as
   `[region_id, forward_cell, lateral_cell, min_z_m, max_z_m]` in
   `config/local_mapping.yaml`.

## Replay and evidence

Replay recorded data with Foxy wall time, timestamp validation disabled, and
output restamping enabled. The Jetson's installed Foxy `ros2 bag play` does not
provide `--clock`:

```bash
ros2 launch wheelchair_navigation local_mapping.launch.py use_sim_time:=false \
  validate_cloud_timestamps:=false restamp_output_with_node_time:=true
ros2 bag play BAG_DIRECTORY \
  --qos-profile-overrides-path \
  "$(ros2 pkg prefix --share wheelchair_navigation)/config/rosbag_reliable_lidar_qos.yaml" \
  --topics /rslidar_points /tf /tf_static
```

The mapper intentionally requires a reliable `/rslidar_points` publisher.
Current AIRY recordings replay reliably; keep the override for compatibility
with older or externally produced bags whose recorded QoS is absent or
best-effort. Replay only the sensor and transform inputs as shown; replaying
recorded output maps and diagnostics alongside the mapper would contaminate
the new results.

Use identical saved RViz viewpoints for raw, **SHADOW ONLY**, mask-rejected
(magenta), low-support (yellow), staircase/halo, and threshold-cell overlays.
Confirm the cyan threshold cells match the intended halo coverage and the
yellow cells match low-support removals. Confirm `/diagnostics` records mask
region count, each region's count, unique rejected points, raw/shadow front
cells, mask-touched/mixed/removed cells, threshold candidates, low-support
cells/points, artifact filter time, processing p95/maximum, and cloud-age p95.

## Acceptance gates

- Raw `/local_obstacles` and `/front_costmap` messages are bit-for-bit unchanged
  with shadow publication enabled or disabled.
- In the empty installed scene, at least 95% of manually labelled near-sensor
  artifact cells disappear. Every mask cell must remain inside its region's
  configured halo footprint.
- Larger structured surfaces visible in the seven source photographs remain.
- With both point thresholds set to one, cell-band exclusion alone is active
  and a mask cell containing any unmasked accepted point stays occupied.
- With the current defaults, every shadow cell requires at least three
  remaining points, while configured mask/halo cells require fifteen. The raw
  maps retain their original one-point sensitivity.
- Magenta mask-rejected and yellow low-support point clouds are disjoint and
  preserve the source cloud frame and timestamp.
- On every labelled obstacle bag, the shadow never changes expected STOP to
  SLOW/CLEAR or expected SLOW to CLEAR.
- Disappearance of the thin horizontal bar at artifact height fails the future
  promotion gate and is documented. Do not compensate by widening a mask.
- Mapping processing p95 stays below 100 ms. Processing maximum and cloud-age
  p95 stay below 150 ms, with no backlog or repeating lag spikes.

The artifact-reduced shadow may be delivered with a documented promotion-gate
failure because it remains diagnostic. Connecting it to shared control requires
a separate reviewed change after all real-obstacle gates pass.
