# Remote Wheelchair Shared-Control Monorepo

This repository hosts software for a shared-control, obstacle-aware powered
wheelchair project. It now includes the Raspberry Pi CAN teleoperation
controller, RoboSense AIRY local mapping, fail-safe operator-intent
supervision, an opt-in Jetson-to-Pi safety link, and isolated Gazebo fixtures.
Physical shared-control remains disabled until the documented validation gates
pass.

## Repository Layout

```text
components/
  can_controller/       Raspberry Pi CAN/RNET teleoperation runtime
  perception/           Host-PC YOLOv8 perception and velocity component
  human_avoidance/      Host-PC pose estimation and human avoidance
  shared_control/       Arbitration between user input, perception, and safety
  communication/        PC-to-Pi command and telemetry protocol code

configs/
  wheelchair/           Wheelchair and CAN controller configuration
  ros2/                 ROS2 node and sensor configuration
  safety/               Shared-control safety policy configuration

docs/
  architecture/         System architecture and ROS2 graph notes
  protocols/            CAN/RNET and PC-to-Pi protocol notes
  setup/                Setup guides and quick-start material
  history/              Historical delivery notes

ros2_ws/src/            ROS2 packages, including wheelchair local mapping
launch/                 System launch scripts for Pi and host PC
scripts/                Setup, deployment, and developer utility scripts
tests/                  Cross-component integration and hardware-in-loop tests
experiments/            Notebooks, logs, and archived experiments
```

## Current Working Component

The existing teleoperation package now lives in:

```text
components/can_controller/
```

Run the keyboard teleoperation entry point from that component:

```bash
cd components/can_controller
python scripts/teleoperate_keyboard.py --config ../../configs/wheelchair/default.yaml
```

See [components/can_controller/README.md](components/can_controller/README.md) for Raspberry Pi, CAN interface, and keyboard-control details.

The local mapping instructions are in
[ros2_ws/src/wheelchair_navigation/README.md](ros2_ws/src/wheelchair_navigation/README.md).
The staged shared-control acceptance checklist is in
[docs/setup/shared_control_validation.md](docs/setup/shared_control_validation.md).

## Current safety boundary

The Jetson supervisor publishes limits, not `cmd_vel` or CAN. The Pi safety
link is disabled unless explicitly configured, and then fails closed on
missing, stale, malformed, or mismatched responses. Simulation motion is
isolated under `/sim` and also defaults off. Autonomous path selection,
reverse assistance, stairs/drop-offs, curbs, and public operation are outside
the current prototype scope. The single-AIRY demo also vetoes left turns
because the chair occludes that sensor sector; it supports forward and bounded
right requests only.
