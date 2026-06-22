# Shared Control Component

Host-PC component that arbitrates between user intent, perception, human avoidance, and safety constraints.

Planned responsibilities:

- Accept user or autonomy command intent.
- Consume obstacle detections and human-avoidance constraints.
- Apply safety policy and degraded-state behavior.
- Publish safe commands for the PC-to-Pi communication layer.

Implementation will live under `src/shared_control/`.
