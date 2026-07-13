# Perception Component

Host-PC component for object perception, human velocity estimation, and local mapping utilities.

Current responsibilities:

- Estimate per-person image-plane velocity from YOLO pose keypoints.
- Provide a simple hemispherical LiDAR local occupancy mapper.

Planned responsibilities:

- Subscribe to ROS2 RGB/RGBD camera topics.
- Run object detection using a configured model.
- Publish detections for shared-control arbitration.
- Keep model weights out of normal Git history.

Implementation lives under `src/perception/`.

## Human Velocity Overlay

`scripts/velocity_overlay.py` consumes Ultralytics YOLO pose results with `stream=True`, computes each person's center from pose keypoints, estimates velocity across the previous 2-3 frames, and draws an arrow on the output frame.

The current estimate is image-plane velocity in pixels per second:

- positive `vx`: person moves right in the image
- negative `vx`: person moves left in the image
- positive `vy`: person moves downward in the image
- negative `vy`: person moves upward in the image

This does not yet compensate for wheelchair/camera ego-motion.

## Setup

```bash
cd components/perception
pip install -r requirements.txt
```

## Run With Camera

```bash
python scripts/velocity_overlay.py --model yolov8m-pose.pt --source 0
```

For a different camera index:

```bash
python scripts/velocity_overlay.py --model yolov8m-pose.pt --source 2
```

For a video file:

```bash
python scripts/velocity_overlay.py --model yolov8m-pose.pt --source path/to/video.mp4
```

Press `q` in the OpenCV window to quit.

## Run Without Display

Use this when testing through SSH or on a headless machine:

```bash
python scripts/velocity_overlay.py --model yolov8m-pose.pt --source 0 --no-window
```

This prints values like:

```text
person_id=1 center=(430.2,288.7) velocity=(61.5,-8.4)px/s speed=62.1px/s latency_total=34.8ms latency_yolo=28.1ms latency_tracker=6.7ms
```

Latency fields:

- `latency_yolo`: Ultralytics-reported preprocessing, inference, and postprocessing time.
- `latency_tracker`: local keypoint extraction, center tracking, velocity calculation, and overlay preparation time.
- `latency_total`: `latency_yolo + latency_tracker`.

## Save Overlay Video

```bash
python scripts/velocity_overlay.py --source path/to/video.mp4 --save outputs/velocity_overlay.mp4
```

## Implementation Notes

- The script uses `model.track(..., stream=True, persist=True)` so YOLO/Ultralytics can provide stable track IDs where possible.
- If track IDs are unavailable, `PersonVelocityTracker` falls back to nearest-center matching between frames.
- Person centers are computed as the confidence-weighted mean of valid keypoints.
- `--history-size 2` estimates velocity between two frames.
- `--history-size 3` uses a short three-sample history, which is usually less twitchy.
- `--arrow-scale` controls visual arrow length without changing the numeric velocity.
- The latency measurement does not include camera sensor exposure time, camera driver buffering, display refresh, or motor command transmission.

## Bring Up RoboSense AIRY LiDAR

This project uses RoboSense's ROS2 driver for the AIRY LiDAR. Build the
RoboSense `rslidar_sdk` workspace separately on the Jetson, then source ROS2
and the SDK workspace before launching the RoboSense driver:

```bash
source /opt/ros/foxy/setup.bash
source /home/jetson-xavier-wheelchair/lidar_workspace/install/setup.bash
ros2 launch rslidar_sdk start.py
```

If your RoboSense workspace is somewhere else, source that workspace instead:

```bash
source /path/to/rslidar_ws/install/setup.bash
```

The RoboSense SDK config should match the AIRY online LiDAR setup:

- `lidar_type: RSAIRY`
- `msg_source: 1`
- `send_point_cloud_ros: true`
- MSOP port `6699`
- DIFOP port `7788`

Check that the point cloud is publishing:

```bash
ros2 topic list
timeout 5 ros2 topic echo /rslidar_points
```

In RViz, add a `PointCloud2` display and select `/rslidar_points`.

## Colorize LiDAR With RealSense RGB

The RGB camera is exposed by the Intel RealSense ROS2 wrapper. Install and
source `realsense2_camera`, then launch the camera:

```bash
sudo apt-get install ros-foxy-realsense2-camera
source /opt/ros/foxy/setup.bash
ros2 launch realsense2_camera rs_launch.py \
  enable_sync:=true \
  align_depth.enable:=true \
  depth_module.profile:=640x480x30 \
  rgb_camera.profile:=640x480x30
```

The color fusion node approximately synchronizes `/rslidar_points`, RealSense
RGB, and depth aligned to RGB. It publishes a colored point cloud while
retaining points that cannot be safely associated in neutral grey:

```bash
cd components/perception
python scripts/colorize_lidar_cloud.py --ros-args \
  --params-file ../../configs/ros2/lidar_camera_fusion.yaml
```

Default topics:

- LiDAR input: `/rslidar_points`
- RGB image: `/camera/color/image_raw`
- Aligned depth: `/camera/aligned_depth_to_color/image_raw`
- RGB intrinsics: `/camera/color/camera_info`
- Colored output: `/rslidar_points_colored`

The marked, rigid sensor mounts currently use this initial transform from the
LiDAR frame (`rslidar`) to the RealSense root frame (`camera_link`):

```bash
ros2 run tf2_ros static_transform_publisher \
  0.42 0.65 1.03 -0.78540 0 0 rslidar camera_link
```

The translation is in metres and the angles are yaw, pitch, and roll in
radians. Refine these initial values against a board visible to both sensors.

Do not publish a manual static transform directly to
`camera_color_optical_frame` while the RealSense node is running. The RealSense
driver already publishes `camera_link -> camera_color_frame ->
camera_color_optical_frame`; adding a second parent for
`camera_color_optical_frame` creates a conflicting TF tree.

In RViz, add a `PointCloud2` display for `/rslidar_points_colored` and set the
color transformer to `RGB8`.

The node colors a projected LiDAR point only when its camera-frame depth agrees
with the aligned RealSense depth. Multiple LiDAR returns at one image pixel are
z-buffered. Its throttled log reports colored, depth-mismatched, invalid-depth,
out-of-image, behind-camera, and z-buffered point counts.

`/rslidar_points_colored` is diagnostic output. Navigation and collision
avoidance must continue to consume the original `/rslidar_points`; RGB-D
association must not create, remove, or relocate safety-critical geometry.

Initial association settings are:

- synchronization slop: `0.08` seconds
- depth tolerance: `max(0.08 m, 0.03 * range)`
- unassociated color: neutral grey `(128, 128, 128)`

Record a repeatable validation bag with the static scene, a board at several
ranges, and a hand occluding the camera while LiDAR sees the background:

```bash
ros2 bag record \
  /rslidar_points \
  /camera/color/image_raw \
  /camera/color/camera_info \
  /camera/aligned_depth_to_color/image_raw \
  /tf /tf_static
```

## Local Navigation Debug Output

After LiDAR points are available in TF, run the local navigation prototype:

```bash
cd components/perception
source /opt/ros/foxy/setup.bash
python scripts/local_navigation.py
```

Default inputs and outputs:

- LiDAR input: `/rslidar_points`
- Target frame: `base_link`
- Costmap output: `/local_costmap`
- Proposed command output: `/proposed_cmd_vel`
- Selected path output: `/local_planner/selected_path`

This node does not command the wheelchair. It publishes debug/proposed motion
only. The command is zero if LiDAR data is stale, the TF lookup fails, or every
sampled trajectory collides with the local costmap.

For early testing without a finalized mount, publish a temporary static TF from
`base_link` to `rslidar` first:

```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link rslidar
```

In RViz, set the fixed frame to `base_link`, then add:

- `Map`: `/local_costmap`
- `Path`: `/local_planner/selected_path`

## Tests

```bash
python -m unittest discover tests
```
