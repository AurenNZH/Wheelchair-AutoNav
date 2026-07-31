# Wheelchair Supervised Navigation

This monorepo contains a LiDAR-first, fail-closed shared-control prototype for
a powered wheelchair. The August 31 MVP allows an operator to request
low-speed forward or bounded-right movement while a Jetson safety supervisor
may permit, slow, or stop it.

Autonomous route selection, reverse assistance, left turns, drop-off
detection, outdoor/public use, and operation without an attendant are outside
the current milestone.

## Layout

```text
components/
  can_controller/       Raspberry Pi keyboard, CAN/RNET, and safety-link runtime
  perception/           Standalone camera/pose experiments, not in the MVP loop

configs/wheelchair/     Raspberry Pi wheelchair configuration
docs/                   Current architecture, protocols, setup, history, roadmap

ros2_ws/src/
  wheelchair_msgs/          OperatorIntent and SafetyEnvelope contracts
  wheelchair_navigation/    AIRY raw local and robot-forward obstacle maps
  wheelchair_shared_control/Fail-closed supervised-motion decisions
  wheelchair_bringup/       Sensors, calibrated TF, mapping, and RViz
  wheelchair_simulation/    Gazebo-only motion and scenario fixtures
  rslidar_sdk, rslidar_msg/ Pinned RoboSense vendor submodules
```

ROS packages remain in `ros2_ws/src` so they build, install, and launch with
normal ROS 2 tooling. Non-ROS Pi and experimental programs remain under
`components`.

## Start Here

- [Current setup and test order](docs/setup/start_here.md)
- [System architecture](docs/architecture/system_architecture.md)
- [AIRY local mapping](ros2_ws/src/wheelchair_navigation/README.md)
- [Shared-control validation gates](docs/setup/shared_control_validation.md)
- [Post-MVP roadmap](docs/roadmap.md)

## Safety Boundary

Physical motion is disabled by default at three independent gates:

- Jetson `enable_motion: false`
- Jetson `enable_udp: false`
- Pi `shared_control.enabled: false`

The ordinary mapper never publishes motion. Gazebo commands are confined to
`/sim/safe_cmd_vel`. Do not enable physical gates before the documented
geometry, network, raised-wheel, stopping-distance, and controlled-obstacle
checks pass.
