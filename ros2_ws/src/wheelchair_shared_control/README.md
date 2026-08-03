# Wheelchair Shared Control

Fail-safe supervisor for operator-requested forward motion. It checks the
requested straight or right-curved swept footprint against `/front_costmap`
and uses observed `/local_obstacles` cells for proximity vetoes. It never
selects a path, publishes `cmd_vel`, or accesses CAN.

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

The normal command above reports `live_control_disabled`, even with valid
maps and intent. The supervisor checks only the straight or gently curved
swept footprint requested by the operator; it does not choose a direction or
start movement. ROS steering is left-positive. The single-AIRY demo permits
`-0.35 <= steering <= 0.0`; left steering stops with
`left_turn_unobserved`, excessive right steering stops with
`right_turn_limit_exceeded`, and reverse remains disabled.

## Recorded-map decision replay

The replay pipeline injects straight-forward operator intent into recorded
`/front_costmap` and `/local_obstacles` messages. It reports the supervisor's
`STOP`, `SLOW`, and `CLEAR` transitions without launching UDP, CAN, Gazebo, or
any velocity adapter. Use a dedicated ROS domain in every terminal:

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

ROS 2 Foxy's bag player does not publish `/clock`, so play only the recorded
maps and remap them through the replay restamper:

```bash
ros2 bag play /path/to/bag \
  --topics /front_costmap /local_obstacles \
  --remap /front_costmap:=/replay/front_costmap \
          /local_obstacles:=/replay/local_obstacles
```

Request or release straight-forward intent from another terminal:

```bash
ros2 param set /operator_intent_injector command forward
ros2 param set /operator_intent_injector command released
```

`forward` expires to `released` after 30 seconds by default. `SLOW` is not an
injector preset: it is a supervisor result when an obstacle enters the
configured slow region. If playback pauses or ends, the restamper stops
publishing and the supervisor returns `stale_map` after its normal timeout.
The restamping path is for decision replay only and cannot validate original
sensor latency. Record suitable derived maps with:

```bash
ros2 bag record /front_costmap /local_obstacles
```

See the
[shared-control validation checklist](../../../../docs/setup/shared_control_validation.md)
for the staged simulation, network, geometry, braking, dummy, and conditional
human-crossing gates.
