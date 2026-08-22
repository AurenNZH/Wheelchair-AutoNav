"""Timestamp validation and receipt timing for mapping point clouds."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class CloudPeriods:
    """Periods between consecutive source stamps and local receipts."""

    source_ms: float
    arrival_ms: float


class CloudTimingTracker:
    """Track source-stamp and receipt periods for a stream of clouds."""

    def __init__(self) -> None:
        self._last_stamp_ns: int | None = None
        self._last_arrival_s: float | None = None

    def record(self, stamp_ns: int, arrival_s: float) -> CloudPeriods:
        periods = CloudPeriods(
            source_ms=period_ms(stamp_ns, self._last_stamp_ns),
            arrival_ms=period_ms(
                arrival_s, self._last_arrival_s, scale=1000.0
            ),
        )
        self._last_stamp_ns = stamp_ns
        self._last_arrival_s = arrival_s
        return periods


def period_ms(current, previous, scale: float = 1e-6) -> float:
    """Return a positive elapsed period in milliseconds, or zero."""

    if previous is None or current <= previous:
        return 0.0
    return float(current - previous) * scale


def cloud_timestamp_error(
    now_ns: int,
    stamp_ns: int,
    max_age_s: float,
    max_future_offset_s: float,
) -> str | None:
    """Return a diagnostic reason for an unusable cloud timestamp."""

    if stamp_ns <= 0:
        return "invalid_lidar_timestamp"
    age_ns = int(now_ns) - int(stamp_ns)
    if age_ns > int(max_age_s * 1e9):
        return "stale_lidar"
    if age_ns < -int(max_future_offset_s * 1e9):
        return "future_lidar"
    return None


def cloud_age_ms(now_ns: int, stamp_ns: int) -> float:
    """Return a non-negative cloud age, treating invalid stamps as zero."""

    if stamp_ns <= 0:
        return 0.0
    return max(0.0, (int(now_ns) - int(stamp_ns)) / 1e6)


__all__ = [
    "CloudPeriods",
    "CloudTimingTracker",
    "cloud_age_ms",
    "cloud_timestamp_error",
    "period_ms",
]
