# Passive Physical-JSM Observation

This stage discovers and validates the physical joystick input without giving
the safety supervisor or observer any physical command authority. The observer
contains no CAN transmit, UDP, ROS publication, gateway-control, or actuator
path.

R-Net joystick position is normally the extended `02000X00#XxYy` frame, where
`X` is the JSM device slot and the two data bytes are signed X/Y positions.
The usual slot-1 frame is `02000100`; X is right-positive and Y is
forward-positive.

## 1. Record the existing topology

Do not create, delete, or restart any gateway rule or service. With the chair
powered and stationary, inspect what is already present:

```bash
ip -details link show can0
ip -details link show can1
cangw -L
ps -ef | grep -E 'cangw|teleoperate_keyboard|wheelchair'
```

It is acceptable for `can1` or `cangw -L` to be absent. The purpose is to
record whether this installation is a single-bus tap, a two-bus in-line
gateway, or something else before designing intervention.

## 2. Locate the physical JSM

Start with the interface believed to be on the physical-JSM side. The observer
requires the interface name explicitly so it cannot guess the control
topology:

```bash
cd /home/jetson-xavier-wheelchair/Wheelchair-AutoNav/components/can_controller
python3 scripts/observe_physical_joystick.py \
  --can-interface can1 --device-slot 1
```

If no samples appear, stop with `Ctrl-C` and repeat on `can0`. Do not run two
instances and do not start keyboard injection during this check. A normal
slot-1 stream should be close to 100 Hz and centered input should report raw
`(0, 0)` or a small repeatable neutral offset.

Example output:

```text
Physical-JSM observer active: interface=can1 frame=02000100#XxYy
RECEIVE ONLY: no CAN frames, UDP packets, ROS messages, or actuator commands are published.
JSM neutral       raw=(   0,   0) ros=(steer=+0.00 forward=0.00 reverse=0.00) rate= 100.0 Hz interval_ms=10.000
JSM forward_right raw=(  20,  40) ros=(steer=-0.20 forward=0.40 reverse=0.00) rate= 100.0 Hz interval_ms=10.000
```

## 3. Capture calibration evidence

With the chair secured according to the existing hardware test procedure,
hold each position for several seconds and return to center between positions:

1. neutral;
2. straight forward;
3. straight reverse;
4. left and right with zero Y;
5. forward-left and forward-right;
6. final neutral.

Record every valid sample:

```bash
python3 scripts/observe_physical_joystick.py \
  --can-interface can1 --device-slot 1 \
  --duration-s 30 --csv /tmp/physical_jsm.csv
```

`--deadzone` changes only the displayed direction label. It never changes the
raw or normalized values written to CSV. Leave it at zero for the first capture
so the data can determine an evidence-based neutral deadzone later. The
observer refuses to overwrite an existing CSV capture.

## Acceptance

- Starting and stopping the observer does not change physical joystick
  behavior.
- The correct interface produces one consistent joystick frame ID near 100 Hz.
- Raw values return reliably to their measured neutral range.
- Forward/reverse and left/right signs match the physical movement.
- The CSV contains no discontinuity attributable to starting the observer.
- CAN error and dropped-frame counters do not increase during observation.

Passing this stage authorizes only the later shadow-intent transport milestone.
It does not authorize suppressing, replacing, or injecting R-Net frames.
