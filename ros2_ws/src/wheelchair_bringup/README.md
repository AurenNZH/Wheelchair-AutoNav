# Wheelchair Bringup

This package owns sensor launch, measured physical transforms, whole-system
launch composition, and the wheelchair RViz profile. It never commands motion.

The default profile starts the right Unitree L2. Mapping, RealSense, and RViz
remain optional:

```bash
source /opt/ros/foxy/setup.bash
source ros2_ws/install/setup.bash
ros2 launch wheelchair_bringup wheelchair.launch.py \
  use_lidar:=true use_camera:=false use_mapping:=true use_rviz:=true
```

The standalone sensor launch is:

```bash
ros2 launch wheelchair_bringup l2_right.launch.py use_rviz:=true
```

Its hardware parameters live in `config/unitree_l2_right.yaml`. The measured
mount is `base_link -> lidar_right_link` at XYZ
`0.330 -0.265 0.320` metres and yaw/pitch/roll
`0.392699082 0 0` radians. Only the installed right sensor transform is
published. Vendor IMU-derived TF is isolated on `/lidar_right/vendor_tf`.

See [L2 bringup](../../../../docs/setup/l2_bringup.md) for Ethernet setup and
publication checks.
