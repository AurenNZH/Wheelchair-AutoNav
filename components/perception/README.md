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

## Tests

```bash
python -m unittest discover tests
```
