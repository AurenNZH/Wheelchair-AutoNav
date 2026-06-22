# Perception YOLO Component

Host-PC component for object and obstacle perception using YOLOv8 or a compatible vision model.

Planned responsibilities:

- Subscribe to ROS2 RGB/RGBD camera topics.
- Run object detection using a configured model.
- Publish detections for shared-control arbitration.
- Keep model weights out of normal Git history.

Implementation will live under `src/perception_yolo/`.
