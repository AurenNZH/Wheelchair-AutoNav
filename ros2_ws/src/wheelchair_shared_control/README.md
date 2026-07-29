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

See the
[shared-control validation checklist](../../../../docs/setup/shared_control_validation.md)
for the staged simulation, network, geometry, braking, dummy, and conditional
human-crossing gates.
