# Jetson–Pi Safety Link

Version 1 is a fail-safe UDP heartbeat between the Raspberry Pi keyboard/CAN
process and the Jetson ROS 2 supervisor. It carries operator intent to the
Jetson and a permitted envelope back to the Pi; it never carries autonomous
trajectories.

- Pi to Jetson intent: UDP `45450`
- Jetson to Pi envelope: UDP `45451`
- Heartbeat: 20 Hz
- Envelope timeout: 200 ms
- Required clear responses before motion: five distinct sequences

Intent example:

```json
{"v":1,"type":"intent","session":"UUID","seq":12,"steering":-0.1,"forward":0.2,"deadman":true}
```

Envelope example:

```json
{"v":1,"type":"envelope","session":"UUID","intent_seq":12,"decision":2,"permitted_forward":0.2,"permitted_steering":-0.1,"reason":"clear","map_age_ms":42.0}
```

In live Nav2 mode, `map_age_ms` is monotonic costmap receipt age. Filtered
LiDAR acquisition age remains a separate Jetson diagnostic and safety gate.

Decisions are `0=STOP`, `1=SLOW`, and `2=CLEAR`. Reverse is unsupported.
ROS steering is left-positive; the Pi keyboard/CAN convention is
right-positive, so the Pi adapter negates steering in both directions.

Both ends validate packet size, version, type, finite bounds, peer address,
session, and sequence. Missing, malformed, stale, out-of-order,
wrong-session, or wrong-sender traffic fails closed. STOP remains latched
until the operator releases the motion key.

The protocol is unauthenticated. Use fixed addresses on an isolated trusted
router LAN and never expose the ports to an untrusted network.
