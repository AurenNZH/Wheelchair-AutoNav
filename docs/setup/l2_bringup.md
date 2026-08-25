# Right Unitree L2 Bringup

The vendor SDK is pinned at `ros2_ws/src/unilidar_sdk2`. Wheelchair-specific
network settings, frames, transforms, launch, and RViz remain in
`wheelchair_bringup` rather than in a vendor-branded wheelchair package.

## Fixed interface

| Item | Value |
|---|---|
| L2 address | `192.168.1.62:6101` |
| Jetson receive address | `192.168.1.2:6201` |
| Point cloud | `/lidar_right/points` |
| Cloud frame | `lidar_right_link` |
| IMU | `/lidar_right/imu` |
| IMU frame | `lidar_right_imu_link` |

The driver uses system timestamps, 18 scans per cloud, and a 0.45–100 m range.
Vendor TF is isolated on `/lidar_right/vendor_tf`.

## Ethernet preflight

```bash
ip -brief address show eth0
sudo ip address add 192.168.1.2/24 dev eth0
ping -c 3 192.168.1.62
ss -lunp | rg ':6201\b' || true
```

Do not add the address twice. Port 6201 must be free before launching.

## Build, launch, and verify

```bash
source /opt/ros/foxy/setup.bash
cd ros2_ws
colcon build --symlink-install \
  --packages-select unitree_lidar_ros2 wheelchair_bringup
source install/setup.bash
ros2 launch wheelchair_bringup l2_right.launch.py use_rviz:=true
```

In another terminal:

```bash
ros2 topic info /lidar_right/points --verbose
ros2 topic hz /lidar_right/points
timeout 3 ros2 topic echo /lidar_right/points sensor_msgs/msg/PointCloud2 --no-arr
```

Passing means the rate remains nonzero, timestamps advance, the TF tree has one
parent for `lidar_right_link`, and moving a target moves the corresponding
points. Leave the sensor running for at least ten minutes before mapping tests.
