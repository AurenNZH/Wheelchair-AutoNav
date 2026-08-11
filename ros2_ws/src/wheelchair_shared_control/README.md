# Wheelchair Shared Control

Fail-safe supervisor for operator-requested forward motion. It checks straight
motion or a correction inside the configured forward cone against
`/front_costmap`.
It never selects a path, publishes `cmd_vel`, or accesses CAN.

Both `enable_motion` and `geometry_calibrated` default to `false`. The UDP
bridge also defaults to disabled. These gates must not be enabled until the
measured chair geometry, elevated-wheel tests, physical cutoff drill, and
stopping-distance acceptance are complete.

Interfaces:

- `/operator_intent` (`wheelchair_msgs/OperatorIntent`)
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
  slow_forward_limit:=0.15
```

Replace the example Pi address with its fixed isolated-LAN address and follow
the [physical-JSM procedure](../../../../docs/setup/physical_joystick_shared_control.md).

The normal command above reports `live_control_disabled`, even with valid maps
and intent. UDP protocol v2 carries normalized left-positive lateral and
forward-positive longitudinal axes plus a semantic class. The supervisor
permits only the symmetric 25-degree forward-correction cone. It checks every
path from straight through the requested correction so the Pi can safely
reduce that correction while applying CLEAR/SLOW caps. Hard turns and reverse
remain classified but return explicit STOP reasons.

The measured 0.80 m base is centred on `base_link`, so its configured forward
and rear extents are both 0.40 m. STOP and SLOW distances are additional
travel-to-contact distances beyond that swept footprint.

## Recorded-map decision replay

The replay pipeline injects straight-forward operator intent into recorded
`/front_costmap` messages. It reports the supervisor's `STOP`, `SLOW`, and
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
maps may have gaps greater than the production limit. The normal shared-control
launch and configuration retain the 0.30-second fail-closed timeout. To test
that stricter behavior, or to restore internal node logs, use:

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

`/local_obstacles` may still be published and inspected in RViz for lidar and
self-filter diagnostics, but it is intentionally outside the supervisor's
forward-only decision contract. Obstacles initially beside or behind the chair
do not veto motion.

See the
[shared-control validation checklist](../../../../docs/setup/shared_control_validation.md)
for the staged simulation, network, geometry, braking, dummy, and conditional
human-crossing gates.
