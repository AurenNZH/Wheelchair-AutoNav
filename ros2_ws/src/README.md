# ROS 2 Workspace Sources

First-party packages:

- `wheelchair_bringup`: sensor drivers, physical transforms, launch composition, and RViz.
- `wheelchair_msgs`: operator-intent and safety-envelope contracts.
- `wheelchair_navigation`: point support filtering and Nav2 costmap production.
- `wheelchair_shared_control`: fail-safe limits for operator-requested motion.
- `wheelchair_simulation`: isolated Gazebo mapping and shared-control fixtures.

`unilidar_sdk2` is the pinned Unitree vendor submodule. Initialize it after
cloning with `git submodule update --init --recursive` from the repository root.
The upstream repository also exposes ROS 1 and standalone example package
descriptors. For a normal workspace build, use:

```bash
colcon build --symlink-install \
  --packages-ignore unitree_lidar_ros unitree_lidar_sdk
```

The physical shared-control and UDP gates are disabled by default. Complete the
[shared-control validation checklist](../../docs/setup/shared_control_validation.md)
before changing them.
