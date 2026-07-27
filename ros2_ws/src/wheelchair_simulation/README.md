# Wheelchair Simulation

Headless Gazebo Classic fixture for mapping-only tests. It models the documented
`base_link -> rslidar` transform, a stationary wheelchair body, a 10 Hz
360-degree ray sensor, and simple walls/obstacles. It does not subscribe to
`cmd_vel`, CAN, or any real actuator interface.

Build the workspace with `gazebo_ros`, `xacro`, and `robot_state_publisher`
installed, then run:

```bash
ros2 launch wheelchair_simulation mapping_sim.launch.py
ros2 launch wheelchair_bringup wheelchair.launch.py use_lidar:=false use_camera:=false use_mapping:=true
```

Use `gui:=true` only on a desktop; the default is headless for Jetson testing.
Gazebo does not reproduce AIRY footrest multipath, so recorded AIRY fixtures
remain the source for reflection-regression tests.
