"""Rolling performance statistics for local mapping."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Mapping

import numpy as np
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue

from wheelchair_navigation.artifact_filter import (
    ArtifactCellSupportStats,
    ArtifactFilterStats,
)
from wheelchair_navigation.costmap import CostmapStats


@dataclass(frozen=True)
class MappingDiagnosticSnapshot:
    """All values needed to format one local-mapping diagnostic status."""

    reason: str
    processing_ms: float
    cloud_age_ms: float
    processing_warn_ms: float
    cloud_age_warn_ms: float
    processed_clouds: int
    rejected_clouds: int
    source_period_ms: float
    arrival_period_ms: float
    artifact_shadow_enabled: bool
    rolling_metrics: Mapping[str, str]
    stats: CostmapStats | None = None
    stage_ms: Mapping[str, float] | None = None
    artifact_stats: ArtifactFilterStats | None = None
    artifact_support_stats: ArtifactCellSupportStats | None = None
    artifact_front_cells: int | None = None


class MappingMetrics:
    """Track recent successful map timings and output rate."""

    def __init__(self, window_samples: int) -> None:
        if window_samples < 1:
            raise ValueError("latency window must contain at least one sample")
        self._processing_ms = deque(maxlen=window_samples)
        self._mapping_ms = deque(maxlen=window_samples)
        self._cloud_age_ms = deque(maxlen=window_samples)
        self._arrivals_s = deque(maxlen=window_samples)

    def record(
        self,
        processing_ms: float,
        arrival_s: float,
        cloud_age_ms: float = 0.0,
        mapping_ms: float | None = None,
    ) -> None:
        self._processing_ms.append(float(processing_ms))
        self._mapping_ms.append(
            float(processing_ms if mapping_ms is None else mapping_ms)
        )
        self._cloud_age_ms.append(float(cloud_age_ms))
        self._arrivals_s.append(float(arrival_s))

    def values(self, lag_spike_ms: float) -> dict[str, str]:
        values = {"effective_rate_hz": "%.3f" % self._rate_hz()}
        if not self._processing_ms:
            return values
        history = np.asarray(self._processing_ms, dtype=np.float32)
        mapping_history = np.asarray(self._mapping_ms, dtype=np.float32)
        age_history = np.asarray(self._cloud_age_ms, dtype=np.float32)
        values.update(
            {
                "latency_window_count": str(history.size),
                "processing_p50_ms": "%.3f"
                % float(np.percentile(history, 50)),
                "processing_p95_ms": "%.3f"
                % float(np.percentile(history, 95)),
                "processing_max_ms": "%.3f" % float(np.max(history)),
                "mapping_p95_ms": "%.3f"
                % float(np.percentile(mapping_history, 95)),
                "cloud_age_p95_ms": "%.3f"
                % float(np.percentile(age_history, 95)),
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


def artifact_shadow_error_reason(exc: Exception) -> str:
    """Map shadow-only failures to stable diagnostic messages."""

    if "frame mismatch" in str(exc):
        return "artifact_shadow_frame_mismatch"
    return "invalid_artifact_shadow_configuration"


def build_mapping_diagnostic_status(
    snapshot: MappingDiagnosticSnapshot,
) -> DiagnosticStatus:
    """Format a local-mapping diagnostic without publishing it."""

    if snapshot.reason != "ok":
        level = DiagnosticStatus.ERROR
        message = snapshot.reason
    elif (
        snapshot.processing_ms > snapshot.processing_warn_ms
        or snapshot.cloud_age_ms > snapshot.cloud_age_warn_ms
    ):
        level = DiagnosticStatus.WARN
        message = "mapping_latency_exceeded"
    else:
        level = DiagnosticStatus.OK
        message = "mapping_current"

    values = {
        "processing_ms": "%.3f" % snapshot.processing_ms,
        "cloud_age_ms": "%.3f" % snapshot.cloud_age_ms,
        "processed_clouds": str(snapshot.processed_clouds),
        "rejected_clouds": str(snapshot.rejected_clouds),
        "source_period_ms": "%.3f" % snapshot.source_period_ms,
        "arrival_period_ms": "%.3f" % snapshot.arrival_period_ms,
    }
    values.update(snapshot.rolling_metrics)
    for key, value in (snapshot.stage_ms or {}).items():
        values[key] = "%.3f" % value
    if snapshot.stats is not None:
        values.update(
            {
                "input_points": str(snapshot.stats.input_points),
                "finite_points": str(snapshot.stats.finite_points),
                "height_range_points": str(
                    snapshot.stats.height_range_points
                ),
                "self_filtered_points": str(
                    snapshot.stats.self_filtered_points
                ),
                "accepted_points": str(snapshot.stats.accepted_points),
                "occupied_cells": str(snapshot.stats.occupied_cells),
                "front_points": str(snapshot.stats.front_points),
                "front_occupied_cells": str(
                    snapshot.stats.front_occupied_cells
                ),
                "raw_front_cells": str(
                    snapshot.stats.front_occupied_cells
                ),
            }
        )
    values["artifact_shadow_enabled"] = str(
        snapshot.artifact_shadow_enabled
    ).lower()
    if snapshot.artifact_stats is not None:
        values.update(
            {
                "artifact_region_count": str(
                    snapshot.artifact_stats.region_count
                ),
                "artifact_unique_rejected_points": str(
                    snapshot.artifact_stats.unique_rejected_points
                ),
                "artifact_filtered_front_cells": str(
                    snapshot.artifact_front_cells
                ),
            }
        )
        for index, count in enumerate(
            snapshot.artifact_stats.per_region_rejected_points
        ):
            values["artifact_region_%d_rejected_points" % index] = str(
                count
            )
    if snapshot.artifact_support_stats is not None:
        support = snapshot.artifact_support_stats
        values.update(
            {
                "artifact_min_points_per_cell": str(
                    support.min_points_per_cell
                ),
                "artifact_global_min_points_per_cell": str(
                    support.global_min_points_per_cell
                ),
                "artifact_configured_halo_cells": str(
                    support.configured_halo_cells
                ),
                "artifact_mask_touched_cells": str(
                    support.mask_touched_cells
                ),
                "artifact_mask_removed_cells": str(
                    support.mask_removed_cells
                ),
                "artifact_mask_mixed_cells": str(
                    support.mask_mixed_cells
                ),
                "artifact_threshold_candidate_cells": str(
                    support.threshold_candidate_cells
                ),
                "artifact_low_support_cells": str(
                    support.low_support_cells
                ),
                "artifact_low_support_points": str(
                    support.low_support_points
                ),
            }
        )

    status = DiagnosticStatus()
    status.level = level
    status.name = "wheelchair_navigation/local_costmap"
    status.hardware_id = "robosense_airy"
    status.message = message
    status.values = [
        KeyValue(key=key, value=value) for key, value in values.items()
    ]
    return status


__all__ = [
    "MappingDiagnosticSnapshot",
    "MappingMetrics",
    "artifact_shadow_error_reason",
    "build_mapping_diagnostic_status",
]
