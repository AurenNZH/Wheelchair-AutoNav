# Wheelchair Simulation

Gazebo Classic fixtures for LiDAR mapping and supervised shared control.
Nothing in this package accesses CAN or a physical actuator. Simulated motion
is published only on `/sim/safe_cmd_vel`.

Use an isolated ROS domain:

```bash
export ROS_DOMAIN_ID=91
source /opt/ros/foxy/setup.bash
source ros2_ws/install/setup.bash
```

## Automated Scenarios

```bash
ros2 launch wheelchair_simulation shared_control_sim.launch.py \
  gui:=false enable_sim_motion:=true operator_mode:=scenario scenario:=all
```

Available individual scenarios:

- `missing_intent`
- `clear_forward`
- `obstacle_slow`
- `obstacle_stop`
- `right_sweep_blocked`
- `moving_dummy_stop`
- `left_unobserved`
- `reverse_disabled`
- `stale_intent`

The runner positions Gazebo fixtures, publishes intent, observes
`/safety_envelope` and `/sim/safe_cmd_vel`, and exits non-zero when the
expected fail-closed result is not sustained. Use interactive mode to inspect
the resulting physical motion and clearance visually.

Current calibration status: `missing_intent` passes end to end. The full suite
currently stops at `clear_forward` because Gazebo's ray model reports part of
the fixed-joint wheelchair model as a near obstacle. This is a simulation
frame/self-return issue, not a reason to weaken the physical surround check.
Resolve it before treating `scenario:=all` as a regression gate.

## Interactive Exploration

```bash
ros2 launch wheelchair_simulation shared_control_sim.launch.py \
  gui:=true enable_sim_motion:=true operator_mode:=keyboard
```

Controls:

- W: straight request
- D: bounded-right request
- A: unsupported-left test
- S: reverse-disabled test
- Space or X: release/stop
- Q: quit

The movement deadman expires after 350 ms unless a motion key is refreshed.

## Safe Defaults

With `operator_mode:=none` and `enable_sim_motion:=false`, the supervisor
remains fail closed and the adapter continuously publishes zero velocity.
`enable_sim_motion` affects only the simulation adapter.

Gazebo validates transforms, mapping interfaces, decision logic, watchdogs,
and simulated motion. It does not reproduce AIRY multipath, dome
contamination, real network delay, real wheelchair braking, caster behaviour,
or reliable detection of hazards below the physical sensor. Physical
validation remains mandatory.
