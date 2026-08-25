# Current File Reference

| Area | Purpose |
|---|---|
| `components/can_controller` | Raspberry Pi CAN/RNET and UDP safety client |
| `components/perception` | Standalone camera experiments |
| `configs/wheelchair/default.yaml` | Pi CAN, gateway, and safety defaults |
| `docs/architecture` | Current Jetson/Pi data flow and boundaries |
| `docs/setup` | L2, Nav2, simulation, and physical validation |
| `ros2_ws/src/unilidar_sdk2` | Pinned vendor L2 SDK |
| `ros2_ws/src/wheelchair_bringup` | Sensor configuration, TF, launch, and RViz |
| `ros2_ws/src/wheelchair_msgs` | ROS interface contracts |
| `ros2_ws/src/wheelchair_navigation` | Point support filtering and Nav2 maps |
| `ros2_ws/src/wheelchair_shared_control` | Swept-path STOP/SLOW/CLEAR supervisor |
| `ros2_ws/src/wheelchair_simulation` | Gazebo sensor, fixtures, and scenarios |

Package-owned ROS assets remain beside their package so `colcon` installs them.
Large bags, build products, and caches remain local and ignored by Git.
