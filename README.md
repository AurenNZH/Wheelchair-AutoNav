# Remote Wheelchair Shared-Control Monorepo

This repository hosts software for a shared-control, obstacle-aware powered wheelchair project. The current working code is the Raspberry Pi CAN teleoperation controller; the repository structure is now prepared for host-PC perception, human avoidance, ROS2 integration, and shared-control development.

## Repository Layout

```text
components/
  can_controller/       Raspberry Pi CAN/RNET teleoperation runtime
  perception/           Host-PC YOLOv8 perception, velocity, and mapping component
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

ros2_ws/src/            ROS2 packages, messages, bringup, and sensor bridges
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

The migrated architecture notes are in [docs/architecture/system_architecture.md](docs/architecture/system_architecture.md).

## Migration Plan

The phased migration checklist is tracked in `MONOREPO_MIGRATION_TODO.md`.

The next recommended steps are to add unit tests around the existing CAN controller behavior, then introduce the ROS2 message contracts that perception, human avoidance, shared control, and communication will share.
