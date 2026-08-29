# Wheelchair Obstacle Avoidance

This package adds low-latency, odometry-free local planning to shared control.
It never commands the wheelchair. For forward, forward-left, and forward-right
intent it asks Nav2 Smac Hybrid-A* for a path to a temporary goal 3.8 m along
the current joystick ray, validates that path, and publishes only a bounded,
short-lived steering suggestion.

The planner uses the current 5 m by 8 m `base_link` costmap (4,000 cells), a
Dubins forward-only model, 1.2 m minimum turning radius, 36 heading bins, and a
30 ms search budget. For initial shadow validation, end-to-end planner results
arriving after 300 ms are discarded; this is a validation tolerance rather
than the latency target. Paths must stay within 0.8 m of the joystick ray, be
no longer than 1.25 times the direct route, make no reverse progress, and
finish within 0.2 m of the goal.

Shared-control modes are `disabled` (default), `shadow`, and `enforce`. Start a
shadow stack with:

```bash
sudo apt-get install ros-foxy-nav2-planner ros-foxy-smac-planner
ros2 launch wheelchair_obstacle_avoidance obstacle_avoidance.launch.py \
  avoidance_mode:=shadow discard_after_ms:=300.0
```

The operator gateway must advertise non-zero authority before enforcement can
alter steering. Forward-left and forward-right assistance may only reduce the
requested correction toward straight by the advertised amount. Straight intent
can receive at most that amount in either direction. Longitudinal magnitude is
never increased or generated. Missing, late, invalid, or rejected plans fall
back to the existing direct STOP/SLOW/CLEAR policy.
