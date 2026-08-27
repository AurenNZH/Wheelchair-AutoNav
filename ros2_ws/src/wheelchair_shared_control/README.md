# Wheelchair Shared Control

The supervisor combines `/operator_intent`, `/nav2_merged_costmap`, and the
successful-filter heartbeat `/lidar_right/filter/source_header` to emit
`/safety_envelope`. It owns STOP/SLOW/CLEAR interpretation, checked-corridor
visualization, and the optional Jetson–Pi UDP safety link. It does not generate
CAN frames or choose a route.

Both `enable_motion` and `geometry_calibrated` default to `false`; UDP also
defaults off:

```bash
ros2 launch wheelchair_shared_control shared_control.launch.py
```

Live `nav2_live` freshness independently checks costmap receipt time and the
original successful-filter sensor stamp. The initial weighted policy treats
costs 1–98 as SLOW and 99–100 as STOP; unknown or invalid geometry fails closed.

## Nav2 rosbag replay

Record the current interfaces:

```bash
ros2 bag record /operator_intent /lidar_right/filter/source_header \
  /nav2_merged_costmap /safety_envelope /shared_control/diagnostics
```

Decision replay accepts Nav2 costmaps only:

```bash
ros2 launch wheelchair_shared_control intent_replay.launch.py
ros2 bag play /path/to/bag --topics /nav2_merged_costmap \
  --remap /nav2_merged_costmap:=/replay/nav2_merged_costmap
```

The replay restamper republishes the recorded grid on `/nav2_merged_costmap` and
uses `map_stamp` freshness. It is for decision regression only and cannot
validate original sensor-to-map latency.

For an older bag, remap its recorded `/nav2_front_costmap` directly to
`/replay/nav2_merged_costmap` during playback. No live compatibility alias is
published.

Follow the
[shared-control validation checklist](../../../../docs/setup/shared_control_validation.md)
before enabling any physical gate.
