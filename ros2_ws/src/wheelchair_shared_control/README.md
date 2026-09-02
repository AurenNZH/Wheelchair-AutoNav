# Wheelchair Shared Control

The supervisor combines `/operator_intent`, `/nav2_merged_costmap`, and the
successful-filter heartbeats to emit `/safety_envelope`. Forward trajectories
retain the right-L2 heartbeat gate. Hard turns require both L2 heartbeats and
check the costs in a 0.45 m pixelated disc centred on `base_link`. It owns
STOP/SLOW/CLEAR interpretation, checked-region visualization, and the optional
Jetson–Pi UDP safety link. It does not generate CAN frames. Its optional
reactive selector compares bounded individual arcs only when the direct policy
returns `nav2_cost_slow`; it does not run or consume a path planner.

Both `enable_motion` and `geometry_calibrated` default to `false`; UDP also
defaults off:

```bash
ros2 launch wheelchair_shared_control shared_control.launch.py
```

`reactive_assistance_mode` has `disabled`, `shadow`, and `enforce` values and
defaults to `disabled`. Shadow evaluates the 0.30 system range without packet
authority. Enforcement also requires each protocol-v3 physical intent to
advertise non-zero `max_steering_assist`; keyboard teleoperation advertises
zero and retains direct behavior. Direct STOP/CLEAR, reverse, hard turns,
invalid evidence, and stale evidence reset assistance immediately.

Live `nav2_live` freshness independently checks costmap receipt time and the
original successful-filter sensor stamp. Physical intent may be at most 1.00 s
old; this remains independent of the Pi's 0.20 s envelope watchdog. The
weighted policy treats costs 1–98
as SLOW and 99–100 as STOP; unknown or invalid geometry fails closed. Hard-turn
CLEAR/SLOW lateral limits default to 0.90/0.60, with longitudinal adjustment
limited to 0.15.

## Nav2 rosbag replay

Record the current interfaces:

```bash
ros2 bag record /operator_intent /lidar_right/filter/source_header \
  /lidar_left/filter/source_header \
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
[shared-control validation checklist](../../../docs/setup/shared_control_validation.md)
before enabling any physical gate.
