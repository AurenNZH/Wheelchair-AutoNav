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
        self._arrivals_s = deque(maxlen=window_samples)

    def record(self, processing_ms: float, arrival_s: float) -> None:
        self._processing_ms.append(float(processing_ms))
        self._arrivals_s.append(float(arrival_s))

    def values(self, lag_spike_ms: float) -> dict[str, str]:
        values = {"effective_rate_hz": "%.3f" % self._rate_hz()}
        if not self._processing_ms:
            return values
        history = np.asarray(self._processing_ms, dtype=np.float32)
        values.update(
            {
                "latency_window_count": str(history.size),
                "processing_p50_ms": "%.3f"
                % float(np.percentile(history, 50)),
                "processing_p95_ms": "%.3f"
                % float(np.percentile(history, 95)),
                "processing_max_ms": "%.3f" % float(np.max(history)),
                "lag_spike_count": str(
                    int(np.count_nonzero(history > lag_spike_ms))
                ),
            }
        )
        return values

    def _rate_hz(self) -> float:
        if len(self._arrivals_s) < 2:
            return 0.0
        elapsed_s = self._arrivals_s[-1] - self._arrivals_s[0]
        return (len(self._arrivals_s) - 1) / max(elapsed_s, 1e-6)


__all__ = ["MappingMetrics"]
