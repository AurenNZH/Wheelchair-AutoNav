# Wheelchair Simulation

Headless Gazebo Classic fixtures for mapping and shared-control tests. They
model the documented `base_link -> rslidar` transform, a 10 Hz 360-degree ray
sensor, wheelchair wheel geometry, a one-metre doorway, narrow/low obstacles,
and an optional moving dummy. Nothing in this package accesses CAN or a real
actuator interface.

Build the workspace with `gazebo_ros`, `xacro`, and `robot_state_publisher`
installed, then run:

```bash
ros2 launch wheelchair_simulation mapping_sim.launch.py
ros2 launch wheelchair_bringup wheelchair.launch.py use_lidar:=false use_camera:=false use_mapping:=true
ros2 launch wheelchair_simulation shared_control_sim.launch.py
```

Use `gui:=true` only on a desktop; the default is headless for Jetson testing.
The shared-control launch publishes only zero simulation velocity by default:

```bash
ros2 launch wheelchair_simulation shared_control_sim.launch.py \
  gui:=false move_dummy:=false enable_sim_motion:=false
```

`enable_sim_motion:=true` affects only `/sim/safe_cmd_vel`. Motion still
requires fresh matching `/operator_intent` and `/safety_envelope` messages, and
STOP/stale/mismatched messages produce zero velocity. The supervisor is
allowed to evaluate motion in this simulation launch, but the ordinary
`wheelchair_shared_control` launch retains both physical gates as false.

Gazebo does not reproduce AIRY footrest multipath, so recorded AIRY fixtures
remain the source for reflection-regression tests.
