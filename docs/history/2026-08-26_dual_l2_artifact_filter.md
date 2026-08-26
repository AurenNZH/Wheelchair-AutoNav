# Dual-L2 Artifact Filter Calibration — 2026-08-26

The 62.152-second `dual_l2_artifact_empty_01` recording contained 509 right
and 499 left raw clouds, approximately 8 Hz per sensor. It confirmed that the
two close-origin artifact populations are asymmetric, so separate `base_link`
rules are used rather than mirroring one calibration.

Each existing point-support filter now performs one additional hard-box test
before support counting. The normal three-point threshold remains global;
eligible cells in the box's 0.10 m XY halo require fifteen surviving points.
Nav2 still receives `/lidar_right/points_filtered` and
`/lidar_left/points_filtered` as independent observation sources.

The checked-in boxes cover the central persistent envelopes measured from the
empty recording. Offline processing of every recorded cloud produced zero
residual occupied cells inside either configured halo. Including CDR
deserialization, PointCloud2 decoding, base-frame transformation, filtering,
and record-preserving repacking, measured processing was:

| Sensor | Mean | p95 | p99 | Maximum |
|---|---:|---:|---:|---:|
| Right | 5.578 ms | 6.925 ms | 9.802 ms | 19.384 ms |
| Left | 5.738 ms | 7.160 ms | 9.278 ms | 12.222 ms |

An isolated ROS graph replay at four times the recorded rate also exercised
the installed launch configuration. Both filter diagnostics reported `ok`
with artifact filtering enabled, both source-heartbeat topics followed their
filtered clouds, and Nav2 continued publishing `/nav2_front_costmap`. The
observed filtered-cloud rates were 17.8 Hz right and 17.2 Hz left while the
costmap published at 6.4 Hz. This accelerated replay is a throughput check;
the percentile timings above are the full-record per-cloud measurements.

This is empty-scene evidence only. The boxes and fifteen-point halos remain
provisional until a board, block, and narrow pole immediately outside their
practical boundaries remain visible in both filtered clouds and the Nav2 map.
Physical shared-control enforcement remains out of scope for this calibration.
