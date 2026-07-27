# ROS2 Workspace Packages

Current packages:

- `wheelchair_navigation`: non-actuating LiDAR local obstacles and costmap.
- `wheelchair_bringup`: top-level sensor, TF, navigation, and RViz launching.
- `wheelchair_simulation`: headless Gazebo Classic mapping fixtures.
- `rslidar_sdk`: pinned RoboSense LiDAR driver submodule.
- `rslidar_msg`: pinned RoboSense packet-message submodule.

Initialize vendor sources after cloning with
`git submodule update --init --recursive` from the repository root.

Planned ROS2 packages:

- `wheelchair_msgs`: custom messages, services, and actions.
- `sensor_bridge`: RGBD and LiDAR topic adapters.
- `wheelchair_description`: optional transforms or robot description.
