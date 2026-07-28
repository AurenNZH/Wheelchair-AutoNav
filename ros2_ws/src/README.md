# ROS2 Workspace Packages

Current packages:

- `wheelchair_msgs`: operator-intent and safety-envelope ROS 2 contracts.
- `wheelchair_navigation`: 360-degree and front-180 LiDAR obstacle maps.
- `wheelchair_shared_control`: fail-safe limits for operator-requested motion.
- `wheelchair_bringup`: top-level sensor, TF, navigation, and RViz launching.
- `wheelchair_simulation`: isolated Gazebo mapping/shared-control fixtures.
- `rslidar_sdk`: pinned RoboSense LiDAR driver submodule.
- `rslidar_msg`: pinned RoboSense packet-message submodule.

Initialize vendor sources after cloning with
`git submodule update --init --recursive` from the repository root.

The physical shared-control and UDP gates are disabled by default. Complete the
[shared-control validation checklist](../../docs/setup/shared_control_validation.md)
before changing them.
