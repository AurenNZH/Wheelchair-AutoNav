#!/usr/bin/env python3
"""Send physical JSM intent to the Jetson and shadow or enforce its envelope."""

import argparse
import csv
from collections import deque
from pathlib import Path
import sys
import time


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from wheelchair_teleop.jsm_observer import (  # noqa: E402
    JsmFrameError,
    PhysicalJsmGatewayObserver,
    joystick_frame_id,
)
from wheelchair_teleop.physical_shared_control import (  # noqa: E402
    StraightPhysicalJsmControl,
)
from wheelchair_teleop.safety_link import (  # noqa: E402
    CLEAR,
    MAX_STEERING_ASSIST,
    SLOW,
    STOP,
    SafetyLink,
)


CSV_FIELDS = (
    "wall_time_s",
    "mode",
    "can_id",
    "input_x",
    "input_y",
    "intent_class",
    "intent_heading_deg",
    "supervisor_decision",
    "reason",
    "would_output_x",
    "would_output_y",
    "forwarded_x",
    "forwarded_y",
    "map_age_ms",
    "round_trip_ms",
    "envelope_age_ms",
    "interval_ms",
    "forwarded_to_controller",
    "forwarded_to_joystick",
    "transform_errors",
)


def _integer(text: str) -> int:
    return int(text, 0)


def _positive_integer(text: str) -> int:
    value = _integer(text)
    if value < 1:
        raise argparse.ArgumentTypeError("value must be at least one")
    return value


def _nonnegative_integer(text: str) -> int:
    value = _integer(text)
    if value < 0:
        raise argparse.ArgumentTypeError("value must be non-negative")
    return value


def _positive_float(text: str) -> float:
    value = float(text)
    if value <= 0.0:
        raise argparse.ArgumentTypeError("value must be positive")
    return value


def _nonnegative_float(text: str) -> float:
    value = float(text)
    if value < 0.0:
        raise argparse.ArgumentTypeError("value must be non-negative")
    return value


def _arguments(argv=None):
    parser = argparse.ArgumentParser(
        description=(
            "Bridge an in-line physical R-Net JSM and exchange classified "
            "intent/safety envelopes with the Jetson."
        )
    )
    parser.add_argument(
        "--mode",
        choices=("shadow", "enforce"),
        default="shadow",
        help="shadow preserves physical commands; enforce replaces them (default: shadow)",
    )
    parser.add_argument(
        "--can-interface",
        required=True,
        help="wheelchair-controller SocketCAN interface (for example can0)",
    )
    parser.add_argument(
        "--gateway-interface",
        required=True,
        help="physical-JSM SocketCAN interface (for example can1)",
    )
    parser.add_argument(
        "--device-slot",
        type=_nonnegative_integer,
        required=True,
        help="physical R-Net JSM slot; this chair was measured as slot 2",
    )
    parser.add_argument("--jetson-address", required=True)
    parser.add_argument(
        "--allowed-jetson-address",
        default=None,
        help="accepted envelope source; defaults to --jetson-address",
    )
    parser.add_argument("--intent-port", type=_positive_integer, default=45450)
    parser.add_argument("--envelope-port", type=_positive_integer, default=45451)
    parser.add_argument("--heartbeat-hz", type=_positive_float, default=20.0)
    parser.add_argument(
        "--envelope-timeout-s", type=_positive_float, default=0.20
    )
    parser.add_argument(
        "--required-clear-envelopes", type=_positive_integer, default=5
    )
    parser.add_argument(
        "--max-assist-ratio",
        type=_nonnegative_float,
        default=0.0,
        help=(
            "maximum steering ratio delegated to Nav2 assistance "
            "(default: 0, direct shared control)"
        ),
    )
    parser.add_argument(
        "--require-release-after-obstacle-stop",
        dest="auto_resume_obstacle_stops",
        action="store_false",
        default=True,
        help=(
            "latch obstacle STOP decisions until joystick release instead of "
            "automatically resuming after fresh non-STOP envelopes"
        ),
    )
    parser.add_argument(
        "--clear-cap",
        type=_positive_integer,
        default=90,
        help="local CLEAR ceiling in raw JSM counts (default: 90)",
    )
    parser.add_argument(
        "--slow-cap",
        type=_positive_integer,
        default=60,
        help="local forward SLOW ceiling in raw JSM counts (default: 60)",
    )
    parser.add_argument(
        "--reverse-cap",
        type=_positive_integer,
        default=65,
        help="local reverse ceiling in raw JSM counts (default: 65)",
    )
    parser.add_argument(
        "--turn-clear-cap",
        type=_positive_integer,
        default=90,
        help="local hard-turn CLEAR ceiling in raw counts (default: 90)",
    )
    parser.add_argument(
        "--turn-slow-cap",
        type=_positive_integer,
        default=60,
        help="local hard-turn SLOW ceiling in raw counts (default: 60)",
    )
    parser.add_argument(
        "--turn-longitudinal-cap",
        type=_nonnegative_integer,
        default=15,
        help="hard-turn longitudinal ceiling in raw counts (default: 15)",
    )
    parser.add_argument(
        "--deadzone",
        type=_nonnegative_integer,
        default=5,
        help="two-axis release deadzone in raw counts (default: 5)",
    )
    parser.add_argument(
        "--forward-cone-deg",
        type=_positive_float,
        default=30.0,
        help="symmetric motion correction half-angle (default: 30 degrees)",
    )
    parser.add_argument("--display-rate-hz", type=_positive_float, default=5.0)
    parser.add_argument("--duration-s", type=_positive_float, default=None)
    parser.add_argument(
        "--csv",
        type=Path,
        default=None,
        help="optional new CSV file for every physical JSM sample",
    )
    return parser.parse_args(argv)


def _decision_name(decision) -> str:
    return {STOP: "STOP", SLOW: "SLOW", CLEAR: "CLEAR"}.get(
        decision, "WAITING"
    )


def _optional_number(value, digits: int = 1):
    if value is None:
        return ""
    return ("%%.%df" % digits) % float(value)


def _rate_hz(timestamps) -> float:
    if len(timestamps) < 2:
        return 0.0
    elapsed = timestamps[-1] - timestamps[0]
    return 0.0 if elapsed <= 0.0 else (len(timestamps) - 1) / elapsed


def main(argv=None) -> int:
    args = _arguments(argv)
    try:
        expected_id = joystick_frame_id(args.device_slot)
        if args.deadzone > 99:
            raise ValueError("deadzone must be in [0, 99]")
        if args.clear_cap > 100:
            raise ValueError("clear-cap must be in [1, 100]")
        if args.slow_cap > args.clear_cap:
            raise ValueError("slow-cap must not exceed clear-cap")
        if args.reverse_cap > 100:
            raise ValueError("reverse-cap must be in [1, 100]")
        if args.turn_clear_cap > 100:
            raise ValueError("turn-clear-cap must be in [1, 100]")
        if args.turn_slow_cap > args.turn_clear_cap:
            raise ValueError("turn-slow-cap must not exceed turn-clear-cap")
        if args.turn_longitudinal_cap > 100:
            raise ValueError("turn-longitudinal-cap must be in [0, 100]")
        if args.forward_cone_deg >= 90.0:
            raise ValueError("forward-cone-deg must be less than 90")
        if args.max_assist_ratio > MAX_STEERING_ASSIST:
            raise ValueError(
                "max-assist-ratio must be in [0, %.2f]"
                % MAX_STEERING_ASSIST
            )
    except ValueError as exc:
        print("Configuration error: %s" % exc, file=sys.stderr)
        return 2

    csv_file = None
    writer = None
    safety_link = None
    gateway = None
    try:
        if args.csv is not None:
            csv_file = args.csv.open("x", newline="")
            writer = csv.DictWriter(csv_file, fieldnames=CSV_FIELDS)
            writer.writeheader()

        safety_link = SafetyLink(
            enabled=True,
            jetson_address=args.jetson_address,
            allowed_jetson_address=(
                args.allowed_jetson_address or args.jetson_address
            ),
            intent_port=args.intent_port,
            envelope_port=args.envelope_port,
            heartbeat_hz=args.heartbeat_hz,
            envelope_timeout_s=args.envelope_timeout_s,
            required_clear_envelopes=args.required_clear_envelopes,
            command_cap=args.clear_cap / 100.0,
            slow_command_cap=args.slow_cap / 100.0,
            reverse_command_cap=args.reverse_cap / 100.0,
            turn_command_cap=args.turn_clear_cap / 100.0,
            slow_turn_command_cap=args.turn_slow_cap / 100.0,
            turn_longitudinal_cap=args.turn_longitudinal_cap / 100.0,
            neutral_deadzone=args.deadzone,
            forward_cone_half_angle_deg=args.forward_cone_deg,
            max_steering_assist=args.max_assist_ratio,
            auto_resume_obstacle_stops=args.auto_resume_obstacle_stops,
        )
        control = StraightPhysicalJsmControl(
            safety_link,
            mode=args.mode,
            neutral_deadzone=args.deadzone,
            forward_cone_half_angle_deg=args.forward_cone_deg,
        )
        gateway = PhysicalJsmGatewayObserver(
            controller_interface=args.can_interface,
            joystick_interface=args.gateway_interface,
            device_slot=args.device_slot,
            jsm_transform=control.transform,
        )
        gateway.open()
    except (OSError, ValueError) as exc:
        if safety_link is not None:
            safety_link.close()
        if csv_file is not None:
            csv_file.close()
        print("Unable to start physical shared control: %s" % exc, file=sys.stderr)
        return 1

    print(
        "Physical shared control active: mode=%s JSM=%s controller=%s "
        "frame=%08X#XxYy obstacle_auto_resume=%s max_assist_ratio=%.3f"
        % (
            args.mode.upper(),
            args.gateway_interface,
            args.can_interface,
            expected_id,
            (
                "disabled (release required)"
                if not args.auto_resume_obstacle_stops
                else "enabled"
            ),
            args.max_assist_ratio,
        ),
        flush=True,
    )
    if args.auto_resume_obstacle_stops:
        print(
            "Obstacle STOP recovery requires %d fresh, distinct, matching "
            "SLOW/CLEAR envelopes; any STOP restarts the count."
            % args.required_clear_envelopes,
            flush=True,
        )
    if args.mode == "shadow":
        print(
            "SHADOW: physical commands pass unchanged; safe output is diagnostic only.",
            flush=True,
        )
    else:
        print(
            "ENFORCE: forward cone=+/-%.1fdeg CLEAR<=%d SLOW<=%d; "
            "turn CLEAR<=%d SLOW<=%d longitudinal<=%d; STOP=0."
            % (
                args.forward_cone_deg,
                args.clear_cap,
                args.slow_cap,
                args.turn_clear_cap,
                args.turn_slow_cap,
                args.turn_longitudinal_cap,
            ),
            flush=True,
        )
    print(
        "No cangw rules, keyboard teleop, observer, or second gateway may run concurrently.",
        flush=True,
    )
    if args.csv is not None:
        print("Recording every physical sample to %s" % args.csv, flush=True)

    started = time.monotonic()
    last_display = 0.0
    last_signature = None
    recent_timestamps = deque()
    display_period_s = 1.0 / args.display_rate_hz

    try:
        while args.duration_s is None or time.monotonic() - started < args.duration_s:
            sample = gateway.receive()
            if sample is None:
                continue
            result = control.last_result
            if result is None:
                continue

            recent_timestamps.append(sample.monotonic_s)
            while (
                recent_timestamps
                and sample.monotonic_s - recent_timestamps[0] > 2.0
            ):
                recent_timestamps.popleft()

            interval_ms = (
                None
                if sample.interval_s is None
                else sample.interval_s * 1000.0
            )
            stats = gateway.stats
            if writer is not None:
                writer.writerow(
                    {
                        "wall_time_s": "%.6f" % sample.wall_time_s,
                        "mode": args.mode,
                        "can_id": "%08X" % sample.can_id,
                        "input_x": result.input_x,
                        "input_y": result.input_y,
                        "intent_class": result.intent_label,
                        "intent_heading_deg": _optional_number(
                            result.heading_deg, 3
                        ),
                        "supervisor_decision": _decision_name(
                            result.supervisor_decision
                        ),
                        "reason": result.reason,
                        "would_output_x": result.would_output_x,
                        "would_output_y": result.would_output_y,
                        "forwarded_x": result.forwarded_x,
                        "forwarded_y": result.forwarded_y,
                        "map_age_ms": _optional_number(result.map_age_ms, 3),
                        "round_trip_ms": _optional_number(
                            result.round_trip_ms, 3
                        ),
                        "envelope_age_ms": _optional_number(
                            result.envelope_age_ms, 3
                        ),
                        "interval_ms": _optional_number(interval_ms, 3),
                        "forwarded_to_controller": stats.forwarded_to_controller,
                        "forwarded_to_joystick": stats.forwarded_to_joystick,
                        "transform_errors": stats.transform_errors,
                    }
                )

            signature = (
                result.supervisor_decision,
                result.reason,
                result.intent_class,
                result.would_output_x,
                result.would_output_y,
                result.forwarded_x,
                result.forwarded_y,
                result.local_stop_latched,
                stats.transform_errors,
            )
            should_display = (
                signature != last_signature
                or sample.monotonic_s - last_display >= display_period_s
            )
            if should_display:
                print(
                    "%s reason=%s intent=%s angle=%s input=(%d,%d) "
                    "safe=(%d,%d) sent=(%d,%d) "
                    "rate=%.1fHz map_ms=%s rtt_ms=%s forwarded=(%d,%d) errors=%d"
                    % (
                        _decision_name(result.supervisor_decision),
                        result.reason,
                        result.intent_label,
                        _optional_number(result.heading_deg),
                        result.input_x,
                        result.input_y,
                        result.would_output_x,
                        result.would_output_y,
                        result.forwarded_x,
                        result.forwarded_y,
                        _rate_hz(recent_timestamps),
                        _optional_number(result.map_age_ms),
                        _optional_number(result.round_trip_ms),
                        stats.forwarded_to_controller,
                        stats.forwarded_to_joystick,
                        stats.transform_errors,
                    ),
                    flush=True,
                )
                if csv_file is not None:
                    csv_file.flush()
                last_display = sample.monotonic_s
                last_signature = signature
    except KeyboardInterrupt:
        print("\nPhysical shared control stopped by operator.", flush=True)
    except (JsmFrameError, OSError) as exc:
        print("Physical shared-control gateway stopped: %s" % exc, file=sys.stderr)
        return 1
    finally:
        if gateway is not None:
            gateway.close()
        if safety_link is not None:
            safety_link.close()
        if csv_file is not None:
            csv_file.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
