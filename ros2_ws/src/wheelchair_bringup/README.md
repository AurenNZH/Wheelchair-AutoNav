# Wheelchair Bringup

This package owns sensor launch, measured physical transforms, whole-system
launch composition, and the wheelchair RViz profile. It never commands motion.

The default profile starts both Unitree L2 sensors. Mapping, RealSense, and RViz
remain optional:

```bash
source /opt/ros/foxy/setup.bash
source ros2_ws/install/setup.bash
ros2 launch wheelchair_bringup wheelchair.launch.py \
  use_lidar:=true use_camera:=false use_mapping:=true use_rviz:=true
```

The standalone dual-sensor launch is:

```bash
ros2 launch wheelchair_bringup dual_l2.launch.py use_rviz:=true
```

The hardware parameters live in `config/unitree_l2_right.yaml` and
`config/unitree_l2_left.yaml`. The measured
mount is `base_link -> lidar_right_link` at XYZ
`0.330 -0.220 0.320` metres and yaw/pitch/roll
`0.479965544 0 0` radians. The symmetric defined left mount is
`base_link -> lidar_left_link` at XYZ `0.330 0.220 0.320` metres and
yaw/pitch/roll `-0.479965544 0 0` radians. The wheelchair URDF publishes both
mount frames and supplies the default RViz RobotModel. With
`use_robot_model:=false`, equivalent static mount transforms are used instead.
Vendor IMU-derived TF is isolated on the per-sensor vendor TF topics.

See [L2 bringup](../../../../docs/setup/l2_bringup.md) for Ethernet setup and
publication checks.
