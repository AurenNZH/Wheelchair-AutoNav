"""Estimate per-person image-plane velocity from YOLO pose keypoints."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Deque, Iterable, Optional

import numpy as np


@dataclass(frozen=True)
class PersonObservation:
    """A single person detection in one frame."""

    center_xy: np.ndarray
    keypoints_xy: np.ndarray
    keypoint_conf: Optional[np.ndarray] = None
    track_id: Optional[int] = None


@dataclass(frozen=True)
class VelocityEstimate:
    """Velocity for one tracked person in image coordinates."""

    track_id: int
    center_xy: np.ndarray
    velocity_xy_px_s: np.ndarray
    speed_px_s: float
    dt_s: float
    history_length: int


@dataclass(frozen=True)
class _TrackSample:
    timestamp_s: float
    center_xy: np.ndarray


class PersonVelocityTracker:
    """Track people between frames and estimate center-of-keypoints velocity.

    Velocity is reported in image-plane pixels per second. Positive x means
    rightward motion in the image, and positive y means downward motion.
    """

    def __init__(
        self,
        history_size: int = 3,
        max_match_distance_px: float = 120.0,
        stale_after_s: float = 1.0,
        smoothing_alpha: float = 0.4,
    ) -> None:
        if history_size < 2:
            raise ValueError("history_size must be at least 2")
        if not 0.0 <= smoothing_alpha <= 1.0:
            raise ValueError("smoothing_alpha must be between 0 and 1")

        self.history_size = history_size
        self.max_match_distance_px = max_match_distance_px
        self.stale_after_s = stale_after_s
        self.smoothing_alpha = smoothing_alpha
        self._tracks: dict[int, Deque[_TrackSample]] = {}
        self._smoothed_velocity: dict[int, np.ndarray] = {}
        self._next_track_id = 1

    def update(
        self,
        observations: Iterable[PersonObservation],
        timestamp_s: float,
    ) -> list[VelocityEstimate]:
        """Update tracks from one frame and return velocity estimates."""

        observation_list = list(observations)
        self._drop_stale_tracks(timestamp_s)

        assignments = self._assign_track_ids(observation_list, timestamp_s)
        estimates: list[VelocityEstimate] = []

        for observation, track_id in assignments:
            history = self._tracks.setdefault(track_id, deque(maxlen=self.history_size))
            history.append(_TrackSample(timestamp_s, observation.center_xy.astype(float)))

            estimate = self._estimate_velocity(track_id, history)
            if estimate is not None:
                estimates.append(estimate)

        return estimates

    def _assign_track_ids(
        self,
        observations: list[PersonObservation],
        timestamp_s: float,
    ) -> list[tuple[PersonObservation, int]]:
        used_track_ids: set[int] = set()
        assignments: list[tuple[PersonObservation, int]] = []

        for observation in observations:
            if observation.track_id is None:
                continue

            track_id = int(observation.track_id)
            assignments.append((observation, track_id))
            used_track_ids.add(track_id)
            self._next_track_id = max(self._next_track_id, track_id + 1)

        unmatched = [obs for obs in observations if obs.track_id is None]
        for observation in unmatched:
            track_id = self._nearest_track_id(
                observation.center_xy,
                timestamp_s,
                used_track_ids,
            )
            if track_id is None:
                track_id = self._next_track_id
                self._next_track_id += 1

            assignments.append((observation, track_id))
            used_track_ids.add(track_id)

        return assignments

    def _nearest_track_id(
        self,
        center_xy: np.ndarray,
        timestamp_s: float,
        used_track_ids: set[int],
    ) -> Optional[int]:
        best_track_id: Optional[int] = None
        best_distance = self.max_match_distance_px

        for track_id, history in self._tracks.items():
            if track_id in used_track_ids or not history:
                continue

            age_s = timestamp_s - history[-1].timestamp_s
            if age_s > self.stale_after_s:
                continue

            distance = float(np.linalg.norm(center_xy - history[-1].center_xy))
            if distance <= best_distance:
                best_distance = distance
                best_track_id = track_id

        return best_track_id

    def _estimate_velocity(
        self,
        track_id: int,
        history: Deque[_TrackSample],
    ) -> Optional[VelocityEstimate]:
        if len(history) < 2:
            return None

        oldest = history[0]
        newest = history[-1]
        dt_s = newest.timestamp_s - oldest.timestamp_s
        if dt_s <= 1e-6:
            return None

        raw_velocity = (newest.center_xy - oldest.center_xy) / dt_s
        previous = self._smoothed_velocity.get(track_id)
        if previous is None:
            velocity = raw_velocity
        else:
            alpha = self.smoothing_alpha
            velocity = alpha * raw_velocity + (1.0 - alpha) * previous

        self._smoothed_velocity[track_id] = velocity
        speed = float(np.linalg.norm(velocity))

        return VelocityEstimate(
            track_id=track_id,
            center_xy=newest.center_xy.copy(),
            velocity_xy_px_s=velocity.copy(),
            speed_px_s=speed,
            dt_s=dt_s,
            history_length=len(history),
        )

    def _drop_stale_tracks(self, timestamp_s: float) -> None:
        stale_ids = [
            track_id
            for track_id, history in self._tracks.items()
            if not history or timestamp_s - history[-1].timestamp_s > self.stale_after_s
        ]
        for track_id in stale_ids:
            self._tracks.pop(track_id, None)
            self._smoothed_velocity.pop(track_id, None)


def observations_from_yolo_result(
    result: object,
    min_keypoint_conf: float = 0.3,
) -> list[PersonObservation]:
    """Extract person centers from one Ultralytics YOLO pose result.

    The function accepts objects yielded by `model(..., stream=True)` or
    `model.track(..., stream=True, persist=True)`.
    """

    keypoints = getattr(result, "keypoints", None)
    if keypoints is None:
        return []

    xy = _to_numpy(getattr(keypoints, "xy", None))
    if xy is None or xy.size == 0:
        return []

    conf = _to_numpy(getattr(keypoints, "conf", None))
    track_ids = _track_ids_from_result(result)

    observations: list[PersonObservation] = []
    for index, person_keypoints in enumerate(xy):
        person_conf = conf[index] if conf is not None and index < len(conf) else None
        center = center_from_keypoints(
            person_keypoints,
            person_conf,
            min_keypoint_conf=min_keypoint_conf,
        )
        if center is None:
            continue

        track_id = track_ids[index] if track_ids is not None and index < len(track_ids) else None
        observations.append(
            PersonObservation(
                center_xy=center,
                keypoints_xy=person_keypoints,
                keypoint_conf=person_conf,
                track_id=track_id,
            )
        )

    return observations


def center_from_keypoints(
    keypoints_xy: np.ndarray,
    keypoint_conf: Optional[np.ndarray] = None,
    min_keypoint_conf: float = 0.3,
) -> Optional[np.ndarray]:
    """Compute a center point from valid human pose keypoints."""

    points = np.asarray(keypoints_xy, dtype=float)
    if points.ndim != 2 or points.shape[1] != 2:
        return None

    valid = np.isfinite(points).all(axis=1)
    valid &= ~np.all(np.isclose(points, 0.0), axis=1)

    weights = None
    if keypoint_conf is not None:
        conf = np.asarray(keypoint_conf, dtype=float)
        valid &= conf >= min_keypoint_conf
        weights = conf[valid]

    valid_points = points[valid]
    if len(valid_points) == 0:
        return None

    if weights is not None and np.sum(weights) > 0:
        return np.average(valid_points, axis=0, weights=weights)

    return np.mean(valid_points, axis=0)


def _to_numpy(value: object) -> Optional[np.ndarray]:
    if value is None:
        return None
    if hasattr(value, "cpu"):
        value = value.cpu()
    if hasattr(value, "numpy"):
        return value.numpy()
    return np.asarray(value)


def _track_ids_from_result(result: object) -> Optional[list[int]]:
    boxes = getattr(result, "boxes", None)
    if boxes is None:
        return None

    ids = _to_numpy(getattr(boxes, "id", None))
    if ids is None:
        return None

    return [int(track_id) for track_id in ids.reshape(-1)]
