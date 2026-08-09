#!/usr/bin/env python3
"""Display and optionally record physical R-Net joystick input without CAN TX."""

import argparse
import csv
from collections import deque
from pathlib import Path
import sys
import time


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from wheelchair_teleop.jsm_observer import (  # noqa: E402
    JsmFrameError,
    PhysicalJsmObserver,
    direction_label,
    joystick_frame_id,
)


CSV_FIELDS = (
    "wall_time_s",
    "can_interface",
    "can_id",
    "x_raw",
    "y_raw",
    "ros_steering",
    "forward",
    "reverse",
    "direction",
    "interval_ms",
)


def _integer(text: str) -> int:
    return int(text, 0)


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


def _arguments(argv=None):
    parser = argparse.ArgumentParser(
        description=(
            "Observe physical R-Net JSM position frames. This program has no "
            "CAN transmit, UDP, ROS, gateway, or actuator path."
        )
    )
    parser.add_argument(
        "--can-interface",
        required=True,
        help="SocketCAN interface connected to the physical JSM (for example can1)",
    )
    parser.add_argument(
        "--device-slot",
        type=_nonnegative_integer,
        default=1,
        help="R-Net JSM device slot in decimal or 0x-prefixed form (default: 1)",
    )
    parser.add_argument(
        "--deadzone",
        type=_nonnegative_integer,
        default=0,
        help="Diagnostic direction deadzone in raw counts; raw/CSV values stay unchanged",
    )
    parser.add_argument(
        "--display-rate-hz",
        type=_positive_float,
        default=5.0,
        help="Maximum repeated console update rate (default: 5)",
    )
    parser.add_argument(
        "--duration-s",
        type=_positive_float,
        default=None,
        help="Optional capture duration; otherwise run until Ctrl-C",
    )
    parser.add_argument(
        "--csv",
        type=Path,
        default=None,
        help="Optional CSV path; every valid physical JSM frame is recorded",
    )
    return parser.parse_args(argv)


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
    except ValueError as exc:
        print("Configuration error: %s" % exc, file=sys.stderr)
        return 2

    csv_file = None
    writer = None
    try:
        if args.csv is not None:
            # Refuse to overwrite earlier calibration evidence.
            csv_file = args.csv.open("x", newline="")
            writer = csv.DictWriter(csv_file, fieldnames=CSV_FIELDS)
            writer.writeheader()

        observer = PhysicalJsmObserver(
            args.can_interface,
            device_slot=args.device_slot,
        )
        observer.open()
    except (OSError, ValueError) as exc:
        if csv_file is not None:
            csv_file.close()
        print("Unable to start physical-JSM observer: %s" % exc, file=sys.stderr)
        return 1

    print(
        "Physical-JSM observer active: interface=%s frame=%08X#XxYy"
        % (args.can_interface, expected_id)
    )
    print(
        "RECEIVE ONLY: no CAN frames, UDP packets, ROS messages, or actuator "
        "commands are published."
    )
    if args.csv is not None:
        print("Recording every valid sample to %s" % args.csv)

    started = time.monotonic()
    last_display = 0.0
    last_direction = None
    recent_timestamps = deque()
    display_period_s = 1.0 / args.display_rate_hz

    try:
        while args.duration_s is None or time.monotonic() - started < args.duration_s:
            sample = observer.receive()
            if sample is None:
                continue

            recent_timestamps.append(sample.monotonic_s)
            while (
                recent_timestamps
                and sample.monotonic_s - recent_timestamps[0] > 2.0
            ):
                recent_timestamps.popleft()

            direction = direction_label(sample, args.deadzone)
            interval_ms = (
                ""
                if sample.interval_s is None
                else "%.3f" % (sample.interval_s * 1000.0)
            )
            if writer is not None:
                writer.writerow(
                    {
                        "wall_time_s": "%.6f" % sample.wall_time_s,
                        "can_interface": args.can_interface,
                        "can_id": "%08X" % sample.can_id,
                        "x_raw": sample.x_raw,
                        "y_raw": sample.y_raw,
                        "ros_steering": "%.3f" % sample.ros_steering,
                        "forward": "%.3f" % sample.forward,
                        "reverse": "%.3f" % sample.reverse,
                        "direction": direction,
                        "interval_ms": interval_ms,
                    }
                )

            should_display = (
                direction != last_direction
                or sample.monotonic_s - last_display >= display_period_s
            )
            if should_display:
                print(
                    "JSM %-13s raw=(%4d,%4d) ros=(steer=%+.2f forward=%.2f "
                    "reverse=%.2f) rate=%6.1f Hz interval_ms=%s"
                    % (
                        direction,
                        sample.x_raw,
                        sample.y_raw,
                        sample.ros_steering,
                        sample.forward,
                        sample.reverse,
                        _rate_hz(recent_timestamps),
                        interval_ms or "first",
                    )
                )
                last_display = sample.monotonic_s
                last_direction = direction
    except KeyboardInterrupt:
        print("\nObservation stopped by operator.")
    except JsmFrameError as exc:
        print("Rejected physical-JSM frame: %s" % exc, file=sys.stderr)
        return 1
    finally:
        observer.close()
        if csv_file is not None:
            csv_file.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
