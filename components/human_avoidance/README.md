# Human Avoidance Component

Host-PC component for pose estimation and human-aware avoidance.

Planned responsibilities:

- Subscribe to RGBD, LiDAR, or pose-estimation topics.
- Estimate human position, pose, and keep-out zones.
- Publish human-avoidance constraints to shared control.
- Provide software-only tests using fixtures or mocked ROS2 messages.

Implementation will live under `src/human_avoidance/`.
