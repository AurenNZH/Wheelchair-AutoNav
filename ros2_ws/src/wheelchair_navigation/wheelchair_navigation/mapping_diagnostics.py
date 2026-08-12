"""Rolling performance statistics for local mapping."""

from __future__ import annotations

from collections import deque

import numpy as np


class MappingMetrics:
    """Track recent successful map timings and output rate."""

    def __init__(self, window_samples: int) -> None:
        if window_samples < 1:
            raise ValueError("latency window must contain at least one sample")
        self._processing_ms = deque(maxlen=window_samples)
        self._mapping_ms = deque(maxlen=window_samples)
        self._cloud_age_ms = deque(maxlen=window_samples)
        self._cloud_arrival_age_ms = deque(maxlen=window_samples)
        self._front_publish_age_ms = deque(maxlen=window_samples)
        self._source_period_ms = deque(maxlen=window_samples)
        self._arrival_period_ms = deque(maxlen=window_samples)
        self._arrivals_s = deque(maxlen=window_samples)
        self._front_publishes_s = deque(maxlen=window_samples)

    def record(
        self,
        processing_ms: float,
        arrival_s: float,
        cloud_age_ms: float = 0.0,
        mapping_ms: float | None = None,
        cloud_arrival_age_ms: float | None = None,
        front_publish_age_ms: float | None = None,
        source_period_ms: float = 0.0,
        arrival_period_ms: float = 0.0,
        front_publish_s: float | None = None,
    ) -> None:
        self._processing_ms.append(float(processing_ms))
        self._mapping_ms.append(
            float(processing_ms if mapping_ms is None else mapping_ms)
        )
        self._cloud_age_ms.append(float(cloud_age_ms))
        self._cloud_arrival_age_ms.append(
            float(
                cloud_age_ms
                if cloud_arrival_age_ms is None
                else cloud_arrival_age_ms
            )
        )
        self._front_publish_age_ms.append(
            float(
                cloud_age_ms
                if front_publish_age_ms is None
                else front_publish_age_ms
            )
        )
        self._source_period_ms.append(float(source_period_ms))
        self._arrival_period_ms.append(float(arrival_period_ms))
        self._arrivals_s.append(float(arrival_s))
        self._front_publishes_s.append(
            float(arrival_s if front_publish_s is None else front_publish_s)
        )

    def values(self, lag_spike_ms: float) -> dict[str, str]:
        values = {"effective_rate_hz": "%.3f" % self._rate_hz()}
        if not self._processing_ms:
            return values
        history = np.asarray(self._processing_ms, dtype=np.float32)
        mapping_history = np.asarray(self._mapping_ms, dtype=np.float32)
        age_history = np.asarray(self._cloud_age_ms, dtype=np.float32)
        arrival_age_history = np.asarray(
            self._cloud_arrival_age_ms, dtype=np.float32
        )
        front_age_history = np.asarray(
            self._front_publish_age_ms, dtype=np.float32
        )
        source_period_history = self._positive_history(
            self._source_period_ms
        )
        arrival_period_history = self._positive_history(
            self._arrival_period_ms
        )
        front_period_history = self._period_history(
            self._front_publishes_s
        )
        values.update(
            {
                "latency_window_count": str(history.size),
                "processing_p50_ms": "%.3f"
                % float(np.percentile(history, 50)),
                "processing_p95_ms": "%.3f"
                % float(np.percentile(history, 95)),
                "processing_p99_ms": "%.3f"
                % float(np.percentile(history, 99)),
                "processing_max_ms": "%.3f" % float(np.max(history)),
                "mapping_p95_ms": "%.3f"
                % float(np.percentile(mapping_history, 95)),
                "cloud_age_p95_ms": "%.3f"
                % float(np.percentile(age_history, 95)),
                "cloud_arrival_age_p95_ms": "%.3f"
                % float(np.percentile(arrival_age_history, 95)),
                "front_publish_age_p95_ms": "%.3f"
                % float(np.percentile(front_age_history, 95)),
                "front_publish_age_p99_ms": "%.3f"
                % float(np.percentile(front_age_history, 99)),
                "front_publish_age_max_ms": "%.3f"
                % float(np.max(front_age_history)),
                "front_publish_rate_hz": "%.3f"
                % self._rate_for(self._front_publishes_s),
                "lag_spike_count": str(
                    int(np.count_nonzero(history > lag_spike_ms))
                ),
            }
        )
        self._add_period_values(values, "source_period", source_period_history)
        self._add_period_values(
            values, "arrival_period", arrival_period_history
        )
        self._add_period_values(values, "front_period", front_period_history)
        return values

    def _rate_hz(self) -> float:
        return self._rate_for(self._arrivals_s)

    @staticmethod
    def _rate_for(samples) -> float:
        if len(samples) < 2:
            return 0.0
        elapsed_s = samples[-1] - samples[0]
        return (len(samples) - 1) / max(elapsed_s, 1e-6)

    @staticmethod
    def _positive_history(samples) -> np.ndarray:
        history = np.asarray(samples, dtype=np.float32)
        return history[history > 0.0]

    @staticmethod
    def _period_history(samples) -> np.ndarray:
        if len(samples) < 2:
            return np.empty(0, dtype=np.float32)
        return np.diff(np.asarray(samples, dtype=np.float64)).astype(
            np.float32
        ) * 1000.0

    @staticmethod
    def _add_period_values(
        values: dict[str, str], prefix: str, history: np.ndarray
    ) -> None:
        if history.size == 0:
            return
        values["%s_p95_ms" % prefix] = "%.3f" % float(
            np.percentile(history, 95)
        )
        values["%s_p99_ms" % prefix] = "%.3f" % float(
            np.percentile(history, 99)
        )
        values["%s_max_ms" % prefix] = "%.3f" % float(np.max(history))


__all__ = ["MappingMetrics"]
