# Unitree L2 bring-up

This component brings up one Unitree L2 on the wheelchair's right footrest.
It is intentionally isolated from the repository's main `ros2_ws`: the current
milestone proves stable raw point-cloud and IMU publication and defines the
measured sensor mounts before adding filtering, Nav2, or shared control.

The official Unitree SDK2 is pinned as a Git submodule under
`ros2_ws/src/unilidar_sdk2`. The wrapper package supplies the
wheelchair-specific Ethernet settings, topic names, frame names, and RViz view
without modifying the vendor source.

## Fixed single-L2 interface

| Item | Value |
| --- | --- |
| Lidar address | `192.168.1.62:6101` |
| Jetson L2 address | `192.168.1.2:6201` |
| Point cloud | `/lidar_right/points` (`sensor_msgs/msg/PointCloud2`) |
| Point-cloud frame | `lidar_right_link` |
| IMU | `/lidar_right/imu` (`sensor_msgs/msg/Imu`) |
| IMU frame | `lidar_right_imu_link` |

Both measured L2 mounting frames are published under `base_link`, although only
the right lidar driver is started:

| Transform | XYZ (m) | ROS yaw, pitch, roll |
| --- | --- | --- |
| `base_link -> lidar_right_link` | `0.330 -0.265 0.320` | `+22.5 0 0` degrees |
| `base_link -> lidar_left_link` | `0.330 +0.265 0.320` | `-22.5 0 0` degrees |

The measured angle is a rotation about ROS Z, which ROS/REP-103 calls yaw. The
vendor's separate IMU-derived TF broadcasts are remapped to
`/lidar_right/vendor_tf`; this prevents them from competing with the fixed
wheelchair mount for ownership of `lidar_right_link`.

The driver uses Ethernet mode, standard 3D field of view, IMU enabled, system
timestamps, 18 scans per cloud, and a 0-100 m range. The upstream driver
publishes reliable, volatile topics with a depth of 10.

`cloud_scan_num` is a batching control, not a hardware sampling-rate control.
On the Jetson, 18 scans produced about 5,357 points at 11.96 Hz; 36 scans
produced about 10,712 points at 5.98 Hz. The default remains 18 to avoid
doubling cloud age and update latency. RViz retains 1.2 seconds of clouds for a
more complete display of the L2's non-repetitive scan pattern; this visualization
persistence does not alter `/lidar_right/points` or downstream processing.

## First checkout

From the repository root:

```bash
git submodule update --init --recursive
```

Install any missing ROS dependencies:

```bash
cd components/UniTree_L2/ros2_ws
source /opt/ros/foxy/setup.bash
rosdep install \
  --from-paths \
  src/unilidar_sdk2/unitree_lidar_ros2/src/unitree_lidar_ros2 \
  src/wheelchair_unitree_l2 \
  --ignore-src -r -y
```

## Preflight

Power the L2 and connect its Ethernet cable directly to the Jetson. The Unitree
factory configuration targets `192.168.1.2`; add that address alongside the
Jetson's existing `192.168.1.102` address if it is not already present:

```bash
ip -brief address show eth0
sudo ip address add 192.168.1.2/24 dev eth0
ping -c 3 192.168.1.62
ss -lunp | rg ':6201\b' || true
```

If `192.168.1.2/24` is already listed, do not run the `ip address add` command
again. This is an additive, temporary address: it leaves `192.168.1.102` and the
Wi-Fi/Pi network unchanged, but must be repeated after reboot. The ping must
succeed and UDP port 6201 must not be owned by another lidar process. Stop any
AIRY/RoboSense launch before continuing.

## Build and launch

Build only the ROS 2 driver and wrapper. The vendor repository also contains
ROS 1 and standalone SDK packages which are not part of this milestone.

```bash
cd components/UniTree_L2/ros2_ws
source /opt/ros/foxy/setup.bash
colcon build --symlink-install \
  --packages-select unitree_lidar_ros2 wheelchair_unitree_l2
source install/setup.bash
ros2 launch wheelchair_unitree_l2 single_l2.launch.py
```

RViz starts by default with `lidar_right_link` as its fixed frame. To run the
driver without RViz:

```bash
ros2 launch wheelchair_unitree_l2 single_l2.launch.py use_rviz:=false
```

The network, topic, frame, range, and scan-count defaults are exposed as launch
arguments. List them with:

```bash
ros2 launch wheelchair_unitree_l2 single_l2.launch.py --show-args
```

## Verify publication

In another terminal:

```bash
cd components/UniTree_L2/ros2_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 topic info /lidar_right/points --verbose
ros2 topic hz /lidar_right/points
```

For a short header and metadata sample without printing the point array:

```bash
timeout 3 ros2 topic echo /lidar_right/points \
  sensor_msgs/msg/PointCloud2 --no-arr
```

Leave the driver and `ros2 topic hz` running for at least ten minutes. Passing
means the rate remains non-zero, stamps continue advancing, RViz continues to
update, and there is no freeze or unexplained publisher loss. Check that the
floor and walls are stable and that moving a test object moves the corresponding
points.

The lidar coordinate convention is +X opposite the cable outlet, +Y 90 degrees
counterclockwise from +X, and +Z upward. Set `publish_mount_tfs:=false` only for
isolated vendor-TF diagnosis; it defaults to true for wheelchair operation.

## Troubleshooting boundary

If the lidar pings but no point cloud appears, first confirm that the driver
reports successful UDP initialization and that port 6201 is free. Packet capture
and SDK diagnostics are appropriate next checks; do not compensate by launching
the old AIRY pipeline or changing Nav2 during this standalone test.
