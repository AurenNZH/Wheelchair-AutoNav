# AIRY Flat-Artifact Filter Decision — 2026-08-05

## Evidence reviewed

The decision used these seven external files, which remain outside Git:

- `lidar_artifacts_floor.png`
- `lidar_artifacts_acrylic.png`
- `lidar_artifacts_rslidarframe_onwheelchair.png`
- `lidar_artifacts_baselinkframe_onwheelchair.png`
- `lidar_floor.jpeg`
- `lidar_acrylic.jpeg`
- `lidar_shielded.jpeg`

Floor and black-acrylic results look materially alike. Shielding removes the
chassis from AIRY's view but leaves small near-sensor streaks. In both RViz
frames the concerning returns appear concentrated in a thin plane close to the
sensor, unlike larger structured surfaces that must remain visible.

Screenshots cannot establish exact sensor-frame XYZ bounds, so no mask bounds
were inferred from the photographs. The initial checked-in mask list was empty.
The provisional geometry later measured from a live raw cloud is recorded in
`2026-08-06_airy_provisional_mask_geometry.md`; it remains diagnostic and still
requires the recorded obstacle-gate evaluation below before any promotion.

## Decision

Implement an opt-in-output, enabled-by-default diagnostic shadow of
`/front_costmap`. It uses thin oriented rectangular prisms in native `rslidar`
coordinates. Membership is evaluated before TF, then applied only to the same
height/range/self-filtered points used by the raw maps.

The shadow is published as `/front_costmap_artifact_filtered`; rejected points
and mask volumes are visual diagnostics. `/local_obstacles`, `/front_costmap`,
their topic contracts, and the supervisor input remain unchanged. A frame or
mask configuration error suppresses the shadow and emits an error diagnostic
without stopping raw mapping.

This change does not authorize promotion to shared control. Promotion requires
a separate review after every real-obstacle gate in the validation protocol
passes. A thin horizontal bar at artifact height is the deliberate worst-case
blind-volume test; disappearance is a failed promotion gate, not a reason to
widen or silently accept the mask.

The oriented-prism representation described here was superseded on 2026-08-07
by the cell-aligned three-region staircase recorded in
`2026-08-06_airy_provisional_mask_geometry.md`. The diagnostic-only and
raw-map invariants remain unchanged.
