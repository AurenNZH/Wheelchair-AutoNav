# Wheelchair Supervised Navigation

This monorepo contains a LiDAR-first, fail-closed shared-control prototype for
a powered wheelchair. The operator requests motion; the Jetson supervisor may
permit, slow, or stop it. Autonomous route selection and unattended operation
remain outside the current milestone.

## Layout

```text
components/
  can_controller/       Raspberry Pi CAN/RNET and safety-link runtime
  perception/           Standalone camera experiments

ros2_ws/src/
  unilidar_sdk2/             Pinned Unitree vendor SDK submodule
  wheelchair_bringup/       L2/camera launch, physical TF, and RViz
  wheelchair_msgs/          OperatorIntent and SafetyEnvelope contracts
  wheelchair_navigation/    L2 support filtering and Nav2 costmap production
  wheelchair_shared_control/Fail-closed STOP/SLOW/CLEAR decisions
  wheelchair_simulation/    Gazebo-only sensor, motion, and scenarios
```

The wheelchair packages retain project ownership; only the SDK directory is
vendor-named. There is one ROS workspace and no vendor-branded wheelchair
package.

## Start Here

- [Build and operating order](docs/setup/start_here.md)
- [L2 bringup](docs/setup/l2_bringup.md)
- [System architecture](docs/architecture/system_architecture.md)
- [Shared-control validation](docs/setup/shared_control_validation.md)

Physical motion remains disabled by default at the Jetson supervisor, UDP
bridge, and Raspberry Pi gateway. Do not enable those gates before the
documented geometry, network, raised-wheel, stopping-distance, and controlled
obstacle checks pass.
