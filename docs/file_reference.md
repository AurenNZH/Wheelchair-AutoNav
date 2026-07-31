# Current File Reference

| Area | Purpose |
|---|---|
| `components/can_controller` | Raspberry Pi keyboard, CAN/RNET, and UDP safety client |
| `components/perception` | Standalone YOLO pose/velocity experiments |
| `configs/wheelchair/default.yaml` | Pi CAN, gateway, safety, and shared-control defaults |
| `docs/architecture` | Current Jetson/Pi data flow and safety boundaries |
| `docs/protocols` | RNET notes and the Jetson–Pi safety heartbeat |
| `docs/setup` | AIRY, simulation, and physical validation procedures |
| `ros2_ws/src/wheelchair_bringup` | Sensor, TF, mapper, and RViz launch |
| `ros2_ws/src/wheelchair_navigation` | Point-cloud decoding and raw obstacle maps |
| `ros2_ws/src/wheelchair_shared_control` | Swept-footprint STOP/SLOW/CLEAR supervisor |
| `ros2_ws/src/wheelchair_simulation` | Gazebo model, fixtures, operator, and scenarios |
| `ros2_ws/src/wheelchair_msgs` | ROS interface contracts |

Package-owned ROS configuration stays beside the package so `colcon` installs
it correctly. Large weights, videos, ROS bags, build products, and caches are
local artifacts and are ignored by Git.
