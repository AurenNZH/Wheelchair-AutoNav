# Wheelchair Simulation

Gazebo Classic fixtures for the current right-L2/Nav2/shared-control pipeline.
The simulated sensor publishes `/lidar_right/points` in `lidar_right_link`, so
simulation uses the same point-support filter, `/nav2_front_costmap`, freshness
heartbeat, and supervisor interfaces as physical operation. Nothing in this
package accesses CAN; simulated motion is confined to `/sim/safe_cmd_vel`.

Use an isolated ROS domain:

```bash
export ROS_DOMAIN_ID=91
source /opt/ros/foxy/setup.bash
source ros2_ws/install/setup.bash
ros2 launch wheelchair_simulation shared_control_sim.launch.py \
  gui:=false enable_sim_motion:=true operator_mode:=scenario scenario:=all
```

Available individual scenarios are `missing_intent`, `clear_forward`,
`obstacle_slow`, `obstacle_stop`, `right_sweep_blocked`,
`moving_dummy_stop`, `left_unobserved`, `reverse_disabled`, and
`stale_intent`.

The runner positions Gazebo fixtures, publishes intent, observes
`/safety_envelope` and `/sim/safe_cmd_vel`, and exits non-zero when the
expected fail-closed result is not sustained.

Current calibration status: `missing_intent` passes end to end. The full suite
currently stops at `clear_forward` because Gazebo's ray model reports part of
the fixed-joint wheelchair model as a near obstacle. This known simulation
self-return must be resolved before treating `scenario:=all` as a regression
gate; it is not a reason to weaken the physical L2/Nav2 safety path.

For interactive exploration:

```bash
ros2 launch wheelchair_simulation shared_control_sim.launch.py \
  gui:=true enable_sim_motion:=true operator_mode:=keyboard
```

Interactive mode uses W/D/A/S for requests, Space or X to release, and Q to
quit. All simulation nodes use ROS simulation time. Physical validation remains
mandatory because Gazebo cannot reproduce real L2 sampling, network delay,
wheelchair braking, caster behavior, or low/drop-off hazard coverage.
