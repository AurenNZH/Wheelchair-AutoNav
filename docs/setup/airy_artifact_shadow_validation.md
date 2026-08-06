# AIRY Artifact Shadow Calibration and Validation

This procedure calibrates and evaluates the diagnostic-only pancake masks. The
raw `/front_costmap` remains the safety supervisor input throughout.

## Recordings

For each case, record 10–20 seconds while stationary and preserve the original
cloud frame and timestamps:

```bash
ros2 bag record /rslidar_points /tf /tf_static /front_costmap \
  /local_obstacles /diagnostics
```

Capture open floor, black acrylic, and the installed shielded wheelchair. Also
capture a wide panel, vertical pole, low block, and thin horizontal bar crossing
every prospective mask at stop, slow, and clear distances. Existing
occupancy-grid-only bags cannot be used to recover sensor-frame height. Retain a
raw AIRY PCAP when practical for a non-blocking RoboSense support report.

Name bags and evidence with UTC timestamps and record target dimensions,
positions, expected STOP/SLOW/CLEAR label, AIRY/hood condition, software commit,
and mask configuration alongside each run.

## Deriving one mask

Do not use PCA or clustering.

1. Select the visible trace endpoints in a sensor-frame top view.
2. Set half-width to the maximum stable lateral spread plus one measured point
   spacing allowance.
3. Set the Z interval to the smallest band containing at least 99% of the trace
   in the installed empty-scene recording.
4. Restrict the segment to the observed artifact. Never extend it to hide
   unrelated noise.
5. Enter the seven values as
   `[start_x, start_y, end_x, end_y, half_width, min_z, max_z]` in
   `config/local_mapping.yaml`.

## Replay and evidence

Replay recorded data with simulated time, timestamp validation disabled, and
output restamping enabled:

```bash
ros2 launch wheelchair_navigation local_mapping.launch.py use_sim_time:=true \
  validate_cloud_timestamps:=false restamp_output_with_node_time:=true
ros2 bag play BAG_DIRECTORY --clock
```

Use identical saved RViz viewpoints for raw, **SHADOW ONLY**, prism-rejected
(magenta), low-support (yellow), mask/XY-halo, and threshold-cell overlays.
Confirm the cyan threshold cells match the intended halo coverage and the
yellow cells match low-support removals. Confirm `/diagnostics` records mask
count, each mask's count, unique prism-rejected points, raw/shadow front cells,
prism-touched/mixed/removed cells, threshold candidates, low-support
cells/points, artifact filter time, processing p95/maximum, and cloud-age p95.

## Acceptance gates

- Raw `/local_obstacles` and `/front_costmap` messages are bit-for-bit unchanged
  with shadow publication enabled or disabled.
- In the empty installed scene, at least 95% of manually labelled near-sensor
  artifact cells disappear. With the default local support rule, no cell
  outside the configured XY mask footprints plus
  `artifact_threshold_halo_m` may change.
- Larger structured surfaces visible in the seven source photographs remain.
- With `artifact_min_points_per_cell:=1`, prism exclusion alone is active and a
  prism-touched cell containing any unmasked accepted point stays occupied.
- With the provisional default of two, only in-scope cells with fewer than two
  remaining accepted points clear. Cells outside the mask-local scope retain
  the raw mapper's one-point sensitivity.
- Magenta prism-rejected and yellow low-support point clouds are disjoint and
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
