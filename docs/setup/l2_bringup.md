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

By default, the launch also publishes the wheelchair URDF through
`robot_state_publisher`. The model owns the measured
`base_link -> lidar_right_link` transform and the symmetric, defined
`base_link -> lidar_left_link` mount transform. Only the right L2 driver is
started; the left frame does not imply that a left cloud is available. With
`use_robot_model:=false`, equivalent static mount transforms are published
instead, avoiding duplicate TF publishers.

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

The default RViz profile displays the wheelchair model. To troubleshoot the
sensor without loading the URDF, launch with `use_robot_model:=false`.

In another terminal:

```bash
ros2 topic info /lidar_right/points --verbose
ros2 topic hz /lidar_right/points
timeout 3 ros2 topic echo /lidar_right/points sensor_msgs/msg/PointCloud2 --no-arr
```

Passing means the rate remains nonzero, timestamps advance, the TF tree has one
parent for `lidar_right_link`, and moving a target moves the corresponding
points. Leave the sensor running for at least ten minutes before mapping tests.

## One-time left-L2 MAC configuration

Factory L2 units may share an Ethernet MAC address. Two devices with the same
MAC cannot coexist reliably on a switched LAN even after assigning unique IP
addresses. The guarded hardware utility assigns the already readdressed left
L2 the locally administered unicast MAC `02:29:ab:7c:00:63`.

Before applying it, power off the right L2, stop every Unitree driver, confirm
that the left L2 responds at `192.168.1.63`, and confirm UDP port 6202 is free.

```bash
cd /home/jetson-xavier-wheelchair/Wheelchair-AutoNav
ping -I eth0 -c 4 192.168.1.63
sudo ss -lunp | rg ':6202'

cmake -S ros2_ws/src/wheelchair_bringup/tools \
  -B /tmp/wheelchair_l2_mac_build
cmake --build /tmp/wheelchair_l2_mac_build \
  --target configure_l2_left_mac -j2

# Dry run: prints the exact endpoints and target MAC without changing hardware.
/tmp/wheelchair_l2_mac_build/configure_l2_left_mac

# Persistent change: run only after checking the dry-run output.
/tmp/wheelchair_l2_mac_build/configure_l2_left_mac --apply
```

Power-cycle only the left L2, then clear the stale neighbour entry and verify:

```bash
sudo ip neigh flush to 192.168.1.63 dev eth0
ping -I eth0 -c 20 192.168.1.63
ip neigh show 192.168.1.63 dev eth0
```

The neighbour entry must show `02:29:ab:7c:00:63` with no material packet
loss. Only then power the right L2 and verify that `.62` retains
`0c:29:ab:7c:00:01`, both pings are stable, and the two MAC addresses remain
distinct.
