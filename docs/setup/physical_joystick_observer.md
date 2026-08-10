# Transparent Physical-JSM Observation

This stage discovers and validates the physical joystick input without giving
the safety supervisor command authority. The PiCAN Duo physically splits the
JSM and controller sides, so observation must also keep R-Net communication
alive. The observer forwards every received CAN frame byte-for-byte in both
directions, while generating no joystick command, UDP packet, or ROS message.

R-Net joystick position is normally the extended `02000X00#XxYy` frame, where
`X` is the JSM device slot and the two data bytes are signed X/Y positions.
This chair's measured slot-2 frame is `02000200`; X is right-positive and Y is
forward-positive.

## 1. Prepare the in-line interfaces

Use the same topology as keyboard teleop: `can0` connects to the wheelchair
controller and `can1` connects to the physical JSM. Bring both interfaces up
at 125 kbit/s before powering or operating the chair:

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 125000
sudo ip link set can0 up
sudo ip link set can1 down
sudo ip link set can1 type can bitrate 125000
sudo ip link set can1 up
ip -brief link show can0 can1
cangw -L
```

`cangw -L` must show no rules. Do not run `teleoperate_keyboard.py`, another
observer, or any other gateway concurrently. Duplicate bidirectional gateways
can loop or duplicate R-Net traffic.

## 2. Start passthrough observation

The command uses the same interface meanings as keyboard teleop:

```bash
cd /home/raspberrywheelchair/Wheelchair-AutoNav-control/components/can_controller/scripts
./observe_physical_joystick.py \
  --can-interface can0 --gateway-interface can1 --device-slot 2
```

Here `--can-interface can0` is the controller side and
`--gateway-interface can1` is the physical-JSM side. Do not swap them merely
to diagnose missing output: the direction determines which frames are trusted
as operator input. A normal slot-1 stream should be close to 100 Hz and
centered input should report raw `(0, 0)` or a small repeatable neutral offset.

Example output:

```text
Physical-JSM observer gateway active: JSM can1 <-> controller can0 frame=02000200#XxYy
TRANSPARENT PASS-THROUGH: CAN frames are forwarded unchanged; no joystick commands, UDP packets, or ROS messages are generated.
JSM neutral       raw=(   0,   0) ros=(steer=+0.00 forward=0.00 reverse=0.00) rate= 100.0 Hz interval_ms=10.000 forwarded=(JSM->ctl:100 ctl->JSM:120)
JSM forward_right raw=(  20,  40) ros=(steer=-0.20 forward=0.40 reverse=0.00) rate= 100.0 Hz interval_ms=10.000 forwarded=(JSM->ctl:200 ctl->JSM:240)
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
./observe_physical_joystick.py \
  --can-interface can0 --gateway-interface can1 --device-slot 2 \
  --duration-s 30 --csv /tmp/physical_jsm.csv
```

`--deadzone` changes only the displayed direction label. It never changes the
raw or normalized values written to CSV. Leave it at zero for the first capture
so the data can determine an evidence-based neutral deadzone later. The
observer refuses to overwrite an existing CSV capture.

## Acceptance

- While the observer runs, the JSM has no DIME error and retains normal manual
  control.
- The physical side produces one consistent joystick frame ID near 100 Hz.
- Both displayed passthrough counters increase continuously.
- Raw values return reliably to their measured neutral range.
- Forward/reverse and left/right signs match the physical movement.
- The CSV contains no discontinuity attributable to starting the observer.
- CAN error and dropped-frame counters do not increase during observation.

Passing this stage authorizes only the later shadow-intent transport milestone.
It does not authorize suppressing, replacing, or injecting R-Net frames.
