# Monorepo Migration TODO

This checklist tracks the planned evolution from the current Raspberry Pi CAN teleoperation package into a larger shared-control wheelchair monorepo.

## Goal Structure

```text
remote-wheelchair/
├── README.md
├── LICENSE
├── .gitignore
├── pyproject.toml                 # optional root tooling: ruff, pytest config, etc.
│
├── docs/
│   ├── architecture/
│   │   ├── system_architecture.md
│   │   ├── ros2_graph.md
│   │   └── safety_model.md
│   ├── setup/
│   │   ├── raspberry_pi_setup.md
│   │   ├── host_pc_setup.md
│   │   └── wheelchair_can_setup.md
│   ├── protocols/
│   │   ├── rnet_can_notes.md
│   │   └── pc_pi_control_protocol.md
│   └── diagrams/
│
├── configs/
│   ├── wheelchair/
│   │   └── default.yaml
│   ├── ros2/
│   │   ├── sensors.yaml
│   │   ├── yolo.yaml
│   │   └── human_avoidance.yaml
│   └── safety/
│       └── shared_control.yaml
│
├── components/
│   ├── can_controller/            # Raspberry Pi runtime
│   │   ├── README.md
│   │   ├── requirements.txt
│   │   ├── pyproject.toml          # optional, if packaged independently
│   │   ├── src/
│   │   │   └── wheelchair_can/
│   │   │       ├── __init__.py
│   │   │       ├── can_interface.py
│   │   │       ├── config.py
│   │   │       ├── joystick_controller.py
│   │   │       ├── keyboard_handler.py
│   │   │       └── safety.py
│   │   ├── scripts/
│   │   │   └── teleoperate_keyboard.py
│   │   └── tests/
│   │
│   ├── perception_yolo/           # Host PC vision model / ROS2 node
│   │   ├── README.md
│   │   ├── requirements.txt
│   │   ├── src/
│   │   │   └── perception_yolo/
│   │   ├── launch/
│   │   ├── tests/
│   │   └── models/
│   │       └── README.md           # explain where weights come from
│   │
│   ├── human_avoidance/           # Host PC pose estimation / avoidance
│   │   ├── README.md
│   │   ├── requirements.txt
│   │   ├── src/
│   │   │   └── human_avoidance/
│   │   ├── launch/
│   │   └── tests/
│   │
│   ├── shared_control/            # Arbitration between user, vision, safety
│   │   ├── README.md
│   │   ├── src/
│   │   │   └── shared_control/
│   │   └── tests/
│   │
│   └── communication/             # PC/Pi networking, messages, telemetry
│       ├── README.md
│       ├── src/
│       │   └── wheelchair_comm/
│       ├── tests/
│       └── protocol.md
│
├── ros2_ws/
│   └── src/
│       ├── wheelchair_msgs/        # custom ROS2 msg/srv/action definitions
│       ├── wheelchair_bringup/     # top-level launch files
│       ├── sensor_bridge/          # RGBD/LiDAR subscriptions/adapters
│       └── wheelchair_description/ # optional URDF/static transforms
│
├── launch/
│   ├── pi/
│   │   └── start_can_controller.sh
│   ├── pc/
│   │   ├── start_perception.sh
│   │   ├── start_human_avoidance.sh
│   │   └── start_shared_control.sh
│   └── start_system.sh
│
├── scripts/
│   ├── setup_pi.sh
│   ├── setup_pc.sh
│   ├── run_tests.sh
│   └── sync_to_pi.sh
│
├── tests/
│   ├── integration/
│   ├── hardware_in_loop/
│   └── fixtures/
│
└── experiments/
    ├── notebooks/
    ├── logs/
    └── archived_tests/
```

## Phase 1: Move Current CAN Teleoperation Into Its Own Component

- [x] Create `components/can_controller/`.
- [x] Create `components/can_controller/src/wheelchair_teleop/`.
- [x] Move the current `wheelchair_teleop/` package into `components/can_controller/src/wheelchair_teleop/`.
- [x] Move `teleoperate_keyboard.py` into `components/can_controller/scripts/`.
- [x] Move `setup_utils.py` into `components/can_controller/scripts/`.
- [x] Move `requirements.txt` into `components/can_controller/requirements.txt`.
- [x] Move `config_default.yaml` into `configs/wheelchair/default.yaml`.
- [x] Keep the package name as `wheelchair_teleop` for now to reduce import churn.
- [x] Update the script import path for the new `src/` layout.
- [x] Add `components/can_controller/README.md` describing Raspberry Pi setup, CAN interface setup, and CLI usage.
- [x] Replace the root `README.md` with a concise monorepo overview.

## Phase 2: Add Tests Around Existing Working Behavior

- [ ] Create `components/can_controller/tests/`.
- [ ] Add unit tests for `SafetyManager` speed clamping, acceleration ramping, inactivity timeout, and frame-rate limiting.
- [ ] Add unit tests for configuration loading and default values.
- [ ] Add tests for CAN command/frame construction where hardware access can be mocked.
- [ ] Add a test command to the CAN controller README.
- [ ] Confirm tests run without requiring wheelchair hardware or a live CAN interface.

## Phase 3: Reorganize Existing Documentation

- [x] Create `docs/architecture/`.
- [x] Move `ARCHITECTURE.md` to `docs/architecture/system_architecture.md`.
- [x] Create `docs/protocols/`.
- [x] Move `PROTOCOL_INTEGRATION.md` to `docs/protocols/pc_pi_control_protocol.md`.
- [x] Move `FILE_REFERENCE.md` to `docs/file_reference.md`.
- [x] Move `START_HERE.md` to `docs/setup/start_here.md`.
- [x] Move `DELIVERY_SUMMARY.md` to `docs/history/delivery_summary.md`.
- [x] Replace the root `README.md` with a concise monorepo overview.
- [x] Add links from the root README to the CAN controller README and system architecture docs.

## Phase 4: Add ROS2 Message and Bringup Foundation

- [ ] Create `ros2_ws/src/wheelchair_msgs/` for custom ROS2 messages, services, and actions.
- [ ] Define message contracts for joystick commands, safe velocity commands, detections, human pose zones, and system status.
- [ ] Create `ros2_ws/src/wheelchair_bringup/` for top-level launch files.
- [ ] Create `ros2_ws/src/sensor_bridge/` for RGBD and LiDAR topic adapters if needed.
- [ ] Document expected ROS2 topics, frames, and QoS settings in `docs/architecture/ros2_graph.md`.
- [ ] Add a minimal ROS2 launch path that can start without wheelchair hardware.

## Phase 5: Add Host PC Perception Components

- [ ] Create `components/perception_yolo/`.
- [ ] Add a local README describing YOLOv8 runtime assumptions, model weights, ROS2 inputs, and ROS2 outputs.
- [ ] Add `components/perception_yolo/src/perception_yolo/`.
- [ ] Add `components/perception_yolo/tests/`.
- [ ] Add `components/perception_yolo/models/README.md` explaining where model weights should be stored or downloaded from.
- [ ] Create `components/human_avoidance/`.
- [ ] Add a local README describing pose estimation, human keep-out zones, and ROS2 integration.
- [ ] Add `components/human_avoidance/src/human_avoidance/`.
- [ ] Add `components/human_avoidance/tests/`.
- [ ] Use recorded frames, small fixtures, or mocked ROS2 messages for early tests.

## Phase 6: Add Shared-Control and Communication Components

- [ ] Create `components/shared_control/`.
- [ ] Add a local README describing arbitration between user intent, obstacle detections, human avoidance, and safety constraints.
- [ ] Add `components/shared_control/src/shared_control/`.
- [ ] Add tests for command arbitration, speed limiting, obstacle stops, and degraded sensor states.
- [ ] Create `components/communication/`.
- [ ] Add a local README describing the PC-to-Pi command link.
- [ ] Add `components/communication/protocol.md`.
- [ ] Decide whether PC-to-Pi communication should use ROS2-native transport, SSH command execution, sockets, or another explicit protocol.

## Phase 7: Add Integration Launching and System Scripts

- [ ] Create `launch/pi/start_can_controller.sh`.
- [ ] Create `launch/pc/start_perception.sh`.
- [ ] Create `launch/pc/start_human_avoidance.sh`.
- [ ] Create `launch/pc/start_shared_control.sh`.
- [ ] Create `launch/start_system.sh` once individual launch paths are stable.
- [ ] Create `scripts/setup_pi.sh`.
- [ ] Create `scripts/setup_pc.sh`.
- [ ] Create `scripts/sync_to_pi.sh` for deploying Pi-side code.
- [ ] Add an integration checklist for starting the full system safely.

## Phase 8: Add Integration and Hardware-in-the-Loop Testing

- [ ] Create `tests/integration/`.
- [ ] Create `tests/hardware_in_loop/`.
- [ ] Add integration tests that run perception, shared control, and communication using mocked CAN output.
- [ ] Add hardware-in-the-loop tests that are clearly marked and skipped by default.
- [ ] Add safety preconditions for any test that can move the wheelchair.
- [ ] Document how to run software-only tests separately from hardware tests.

## Phase 9: Manage Large Artifacts and Experiments

- [ ] Create `experiments/notebooks/`.
- [ ] Create `experiments/logs/`.
- [ ] Create `experiments/archived_tests/`.
- [ ] Decide whether to use Git LFS, DVC, GitHub Releases, or external storage for model weights, datasets, ROS bags, and large logs.
- [ ] Add `.gitignore` rules for model weights, generated logs, bag files, caches, and virtual environments.
- [ ] Add download or setup instructions for any required pretrained models.

## Design Principles To Preserve

- [ ] Keep the Raspberry Pi CAN controller deterministic, boring, and safety-focused.
- [ ] Keep heavyweight perception, ROS2, and shared-control logic on the host PC.
- [ ] Keep each component independently runnable and testable.
- [ ] Avoid requiring wheelchair hardware for ordinary unit tests.
- [ ] Treat hardware-in-the-loop tests as explicit, opt-in procedures.
- [ ] Do not commit large model weights, ROS bags, datasets, or experiment logs directly to normal Git history.
