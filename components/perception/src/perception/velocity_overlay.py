"""Run YOLO pose tracking and draw per-person velocity arrows."""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO

from perception.velocity_tracker import (
    PersonVelocityTracker,
    VelocityEstimate,
    observations_from_yolo_result,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Estimate human direction of travel from YOLO pose keypoints."
    )
    parser.add_argument(
        "--model",
        default="yolov8m-pose.pt",
        help="YOLO pose model path or Ultralytics model name.",
    )
    parser.add_argument(
        "--source",
        default="0",
        help="Camera index, video path, or stream URL.",
    )
    parser.add_argument(
        "--conf",
        type=float,
        default=0.4,
        help="YOLO confidence threshold.",
    )
    parser.add_argument(
        "--history-size",
        type=int,
        default=3,
        help="Number of center samples used for velocity estimation.",
    )
    parser.add_argument(
        "--min-keypoint-conf",
        type=float,
        default=0.3,
        help="Minimum keypoint confidence used for center calculation.",
    )
    parser.add_argument(
        "--max-match-distance-px",
        type=float,
        default=120.0,
        help="Nearest-centroid fallback match distance when YOLO track IDs are absent.",
    )
    parser.add_argument(
        "--arrow-scale",
        type=float,
        default=0.15,
        help="Scale factor from pixels/second velocity to arrow length.",
    )
    parser.add_argument(
        "--no-window",
        action="store_true",
        help="Print velocities without opening an OpenCV display window.",
    )
    parser.add_argument(
        "--save",
        type=Path,
        default=None,
        help="Optional path for saving the overlay video.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    model = YOLO(args.model)
    source = _parse_source(args.source)

    tracker = PersonVelocityTracker(
        history_size=args.history_size,
        max_match_distance_px=args.max_match_distance_px,
    )

    writer: cv2.VideoWriter | None = None
    result_stream = model.track(
        source=source,
        stream=True,
        persist=True,
        conf=args.conf,
        classes=[0],
        verbose=False,
    )

    try:
        for result in result_stream:
            processing_start_s = time.perf_counter()
            timestamp_s = time.perf_counter()
            observations = observations_from_yolo_result(
                result,
                min_keypoint_conf=args.min_keypoint_conf,
            )
            estimates = tracker.update(observations, timestamp_s)

            frame = result.plot()
            yolo_latency_ms = yolo_result_latency_ms(result)
            local_latency_ms = (time.perf_counter() - processing_start_s) * 1000.0
            total_latency_ms = yolo_latency_ms + local_latency_ms

            draw_velocity_overlay(
                frame,
                estimates,
                arrow_scale=args.arrow_scale,
                total_latency_ms=total_latency_ms,
            )
            print_velocity_estimates(
                estimates,
                yolo_latency_ms=yolo_latency_ms,
                local_latency_ms=local_latency_ms,
                total_latency_ms=total_latency_ms,
            )

            if args.save is not None:
                writer = _ensure_writer(writer, args.save, frame)
                writer.write(frame)

            if not args.no_window:
                cv2.imshow("YOLO person velocity", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
    finally:
        if writer is not None:
            writer.release()
        if not args.no_window:
            cv2.destroyAllWindows()

    return 0


def draw_velocity_overlay(
    frame: np.ndarray,
    estimates: list[VelocityEstimate],
    arrow_scale: float = 0.15,
    total_latency_ms: float = 0.0,
) -> None:
    """Draw velocity arrows and labels directly on a frame."""

    for estimate in estimates:
        start = estimate.center_xy.astype(int)
        end = start + (estimate.velocity_xy_px_s * arrow_scale).astype(int)

        start_tuple = (int(start[0]), int(start[1]))
        end_tuple = (int(end[0]), int(end[1]))

        cv2.arrowedLine(
            frame,
            start_tuple,
            end_tuple,
            color=(0, 255, 255),
            thickness=2,
            tipLength=0.25,
        )
        label = (
            f"id {estimate.track_id} "
            f"vx={estimate.velocity_xy_px_s[0]:.0f} "
            f"vy={estimate.velocity_xy_px_s[1]:.0f} px/s "
            f"lat={total_latency_ms:.1f}ms"
        )
        cv2.putText(
            frame,
            label,
            (start_tuple[0] + 8, start_tuple[1] - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 255),
            1,
            cv2.LINE_AA,
        )


def print_velocity_estimates(
    estimates: list[VelocityEstimate],
    yolo_latency_ms: float,
    local_latency_ms: float,
    total_latency_ms: float,
) -> None:
    for estimate in estimates:
        print(
            "person_id={id} center=({cx:.1f},{cy:.1f}) "
            "velocity=({vx:.1f},{vy:.1f})px/s speed={speed:.1f}px/s "
            "latency_total={total:.1f}ms "
            "latency_yolo={yolo:.1f}ms latency_tracker={local:.1f}ms".format(
                id=estimate.track_id,
                cx=estimate.center_xy[0],
                cy=estimate.center_xy[1],
                vx=estimate.velocity_xy_px_s[0],
                vy=estimate.velocity_xy_px_s[1],
                speed=estimate.speed_px_s,
                total=total_latency_ms,
                yolo=yolo_latency_ms,
                local=local_latency_ms,
            )
        )


def yolo_result_latency_ms(result: object) -> float:
    """Return Ultralytics-reported latency for one result, if available."""

    speed = getattr(result, "speed", None)
    if not isinstance(speed, dict):
        return 0.0

    return float(
        speed.get("preprocess", 0.0)
        + speed.get("inference", 0.0)
        + speed.get("postprocess", 0.0)
    )


def _parse_source(source: str) -> int | str:
    return int(source) if source.isdigit() else source


def _ensure_writer(
    writer: cv2.VideoWriter | None,
    output_path: Path,
    frame: np.ndarray,
) -> cv2.VideoWriter:
    if writer is not None:
        return writer

    output_path.parent.mkdir(parents=True, exist_ok=True)
    height, width = frame.shape[:2]
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    created = cv2.VideoWriter(str(output_path), fourcc, 30.0, (width, height))
    if not created.isOpened():
        raise RuntimeError(f"Failed to open video writer: {output_path}")
    return created


if __name__ == "__main__":
    raise SystemExit(main())
