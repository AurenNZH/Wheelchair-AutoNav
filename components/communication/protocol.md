# PC-to-Pi Protocol

Version 1 is a fail-safe UDP heartbeat between the Raspberry Pi keyboard/CAN
process and the Jetson ROS 2 supervisor. It carries operator intent to the
Jetson and a permitted envelope back to the Pi. It does not carry autonomous
trajectories.

Default ports:

- Pi to Jetson intent: UDP `45450`
- Jetson to Pi envelope: UDP `45451`

Intent packets are UTF-8 JSON no larger than 1024 bytes:

```json
{"v":1,"type":"intent","session":"UUID","seq":12,"steering":0.1,"forward":0.2,"deadman":true}
```

Envelope packets use the same session and acknowledge one intent sequence:

```json
{"v":1,"type":"envelope","session":"UUID","intent_seq":12,"decision":2,"permitted_forward":0.2,"permitted_steering":0.1,"reason":"clear","map_age_ms":42.0}
```

Decisions are `0=STOP`, `1=SLOW`, and `2=CLEAR`. A STOP packet is invalid if it
permits non-zero motion. Normalized steering is `[-1, 1]`; forward and
permitted forward are `[0, 1]`. Reverse is deliberately absent.

Both ends validate version, type, field types, finite bounds, session, sequence,
packet size, and the configured peer IP. The Pi sends at 20 Hz, stops after
200 ms without a fresh matching envelope, and requires five distinct accepted
intent sequences before allowing a command. A STOP is latched until the
operator releases the motion key. Malformed, stale, out-of-order,
wrong-session, or wrong-sender traffic fails closed.

This protocol is not encrypted or authenticated. Use static addresses on an
isolated trusted LAN, configure both peer allowlists, and do not expose these
ports to an untrusted network.
