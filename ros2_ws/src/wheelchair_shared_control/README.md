# Wheelchair Shared Control

Fail-safe supervisor for operator-requested motion. It checks straight forward
motion or a correction inside the configured forward cone against the weighted
Nav2 `/nav2_front_costmap`. Reverse inside the matching cone is not monitored
by a rear map and is therefore always limited to SLOW.
It never selects a path, publishes `cmd_vel`, or accesses CAN.

Both `enable_motion` and `geometry_calibrated` default to `false`. The UDP
bridge also defaults to disabled. These gates must not be enabled until the
measured chair geometry, elevated-wheel tests, physical cutoff drill, and
stopping-distance acceptance are complete.

Interfaces:

- `/operator_intent` (`wheelchair_msgs/OperatorIntent`)
- `/artifact_filter/source_header` (`std_msgs/Header`)
- `/safety_envelope` (`wheelchair_msgs/SafetyEnvelope`)
- `/shared_control/diagnostics` (`diagnostic_msgs/DiagnosticArray`)
- UDP intent port `45450`, envelope port `45451` when explicitly enabled

Run the fail-closed software:

```bash
ros2 launch wheelchair_shared_control shared_control.launch.py
```

For the separately gated physical-JSM test, the live launch
options are explicit rather than stored as enabled defaults:

```bash
ros2 launch wheelchair_shared_control shared_control.launch.py \
  enable_motion:=true geometry_calibrated:=true enable_udp:=true \
  pi_address:=192.168.1.20 allowed_pi_address:=192.168.1.20 \
  slow_forward_limit:=0.40
```

Replace the example Pi address with its fixed isolated-LAN address and follow
the [physical-JSM procedure](../../../../docs/setup/physical_joystick_shared_control.md).

The normal command above reports `live_control_disabled`, even with valid maps
and intent. UDP protocol v2 carries normalized left-positive lateral and
forward-positive longitudinal axes plus a semantic class. The supervisor
permits the symmetric 25-degree forward- and reverse-correction cones. It
checks every forward path from straight through the requested correction so
the Pi can safely reduce that correction while applying CLEAR/SLOW caps.
Reverse returns `reverse_unmonitored_slow` with a 0.40 magnitude limit without
consulting the front map. Hard turns remain classified and return STOP.

The measured 0.80 m by 0.70 m chair footprint is configured authoritatively in
Nav2. STOP and SLOW distances are trajectory lookahead distances. The cost
policy samples prospective robot-centre poses because Nav2 inflation already
incorporates configuration-space clearance; applying the chair footprint a
second time would double count it.

The initial weighted policy treats costs `1..98` as the SLOW band and costs
`99..100` as the STOP band. A STOP-band cost within 0.70 m produces STOP; any
nonzero cost within 1.20 m produces SLOW. Unknown cost `-1`, invalid geometry,
or a trajectory leaving the grid fails closed. The thresholds are launch
arguments for shadow calibration:

```bash
ros2 launch wheelchair_shared_control shared_control.launch.py \
  enable_motion:=true geometry_calibrated:=true \
  slow_cost_threshold:=1 stop_cost_threshold:=99
```

This enables supervisor decisions only; the supervisor still has no actuator
interface. For the current shadow stage, keep the Pi gateway non-actuating.
Foxy's Nav2 OccupancyGrid does not preserve LiDAR acquisition time. Live mode
therefore requires both a recent `/nav2_front_costmap` arrival and a recent
successful-filter heartbeat carrying the original AIRY stamp. Both default to
a 0.50-second limit. `map_age_ms` is explicitly receipt age in this mode; it is
not an exact LiDAR-to-map latency measurement.

For a weighted shadow capture, enable inflation in the navigation launch and
record the decision evidence on the Jetson:

```bash
ros2 bag record /operator_intent /artifact_filter/source_header \
  /nav2_front_costmap \
  /safety_envelope /shared_control/diagnostics
```

Exercise a clear scene, a graded-cost obstacle that should produce SLOW, an
inscribed/lethal obstacle that should produce STOP, and shallow corrections
on both sides. Diagnostics report `maximum_path_cost`, nearest slow/stop cost
distances, both thresholds, and `path_cost_valid`. A side obstacle outside all
sampled correction trajectories must not change CLEAR.

## Recorded-map decision replay

The replay pipeline explicitly injects straight-forward operator intent into
legacy recorded `/front_costmap` messages. It reports the supervisor's
`STOP`, `SLOW`, and
`CLEAR` transitions without launching UDP, CAN, Gazebo, or any velocity
adapter. Use a dedicated ROS domain in every terminal:

```bash
export ROS_DOMAIN_ID=92
export ROS_LOCALHOST_ONLY=1
source /opt/ros/foxy/setup.bash
source ros2_ws/install/setup.bash
```

Start the decision-only nodes:

```bash
ros2 launch wheelchair_shared_control intent_replay.launch.py
```

The launch console is intentionally limited to three pipeline states:

- `[INTENT]` confirms whether the injector is waiting, released, or requesting
  forward motion.
- `[MAP]` confirms whether the recorded front map is waiting, ready, or stale.
- `[DECISION]` reports only supervisor state changes (`STOP`, `SLOW`, or
  `CLEAR`).

The replay-only map timeout defaults to 2.0 seconds because low-rate recorded
maps may have gaps greater than the production limit. Replay explicitly uses
`legacy_map_stamp`; the normal launch uses the dual-watchdog `nav2_live` mode
with a 0.50-second map receipt limit. To test a stricter replay timeout, or to
restore internal node logs, use:

```bash
ros2 launch wheelchair_shared_control intent_replay.launch.py \
  max_map_age_s:=0.30 internal_log_level:=info
```

ROS 2 Foxy's bag player does not publish `/clock`, so play only the recorded
front map and remap it through the replay restamper:

```bash
ros2 bag play /path/to/bag \
  --topics /front_costmap \
  --remap /front_costmap:=/replay/front_costmap
```

Request or release straight-forward intent from another terminal:

```bash
ros2 param set /operator_intent_injector command forward
ros2 param set /operator_intent_injector command released
```

`forward` expires to `released` after 30 seconds by default. `SLOW` is not an
injector preset: it is a supervisor result when an obstacle enters the
configured slow region. If playback pauses or ends, the restamper stops
publishing and the supervisor returns `stale_map` after the selected replay
timeout.
The restamping path is for decision replay only and cannot validate original
sensor latency. Record suitable derived maps with:

```bash
ros2 bag record /front_costmap
```

Binary `0/100` recordings remain useful for transport and fail-closed
regression, but they do not contain Nav2's inflation gradient and are no
longer distance-calibration evidence for the weighted production policy.

`/local_obstacles` may still be published and inspected in RViz for lidar and
self-filter diagnostics, but it is intentionally outside the supervisor's
forward decision contract. Obstacles initially beside or behind the chair do
not veto forward motion, and reverse has no obstacle-map protection.

See the
[shared-control validation checklist](../../../../docs/setup/shared_control_validation.md)
for the staged simulation, network, geometry, braking, dummy, and conditional
human-crossing gates.
