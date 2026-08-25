# Perception Component

Host-PC component for object perception and human velocity estimation.

Current responsibilities:

- Estimate per-person image-plane velocity from YOLO pose keypoints.

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
python scripts/velocity_overlay.py --source 0
```

For a different camera index:

```bash
python scripts/velocity_overlay.py --source 2
```

For a video file:

```bash
python scripts/velocity_overlay.py --source path/to/video.mp4
```

Press `q` in the OpenCV window to quit.

## Run Without Display

Use this when testing through SSH or on a headless machine:

```bash
python scripts/velocity_overlay.py --source 0 --no-window
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

## Bring Up the Navigation L2

The Unitree SDK is pinned in the main ROS 2 workspace. Initialize submodules and
build before launching:

```bash
source /opt/ros/foxy/setup.bash
cd /home/jetson-xavier-wheelchair/Wheelchair-AutoNav
git submodule update --init --recursive
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch wheelchair_bringup l2_right.launch.py
```

Check that the point cloud is publishing:

```bash
ros2 topic list
timeout 5 ros2 topic echo /lidar_right/points --no-arr
```

In RViz, add a `PointCloud2` display and select `/lidar_right/points`.

## RealSense RGB-D Geometry

The RealSense ROS2 wrapper publishes different representations with different
purposes:

- `/camera/depth/image_rect_raw` is the depth image published by the installed
  L515 wrapper (some wrapper versions call it `/camera/depth/image_raw`).
- `/camera/depth/color/points` is the camera-derived colored `PointCloud2`.
- `/lidar_right/points` is the independent L2 `PointCloud2`.

Recoloring L2 points with the camera does not add navigation geometry.
Navigation starts with `/lidar_right/points`; after the LiDAR-only baseline passes, the RealSense
cloud can contribute independent obstacle evidence to the same local costmap.

The marked sensor mounts are direct children of `base_link`:

```bash
base_link -> lidar_right_link: 0.330 -0.265 0.320 0.392699082 0 0
base_link -> camera_link: -0.360  0.265 1.300 0.00000 0 0
```

These poses use the ROS mobile-base convention: `base_link` X points forward,
Y points left, and Z points up. The RealSense `camera_link` X axis is forward.

The RealSense driver owns the transforms below `camera_link`. Never publish a
second parent directly to `camera_color_optical_frame`.

## Tests

```bash
python3 -m unittest discover tests
```
