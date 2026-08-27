"""Diagnostic formatting for shared-control safety decisions."""

from __future__ import annotations

from dataclasses import dataclass

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue

from wheelchair_shared_control.freshness import NAV2_LIVE
from wheelchair_shared_control.models import (
    CLEAR,
    SafetyConfig,
    SafetyDecision,
)


@dataclass(frozen=True)
class SafetyDiagnosticSnapshot:
    """Inputs required to format one supervisor diagnostic status."""

    decision: SafetyDecision
    config: SafetyConfig
    map_age_ms: float
    processing_ms: float
    freshness_mode: str = NAV2_LIVE
    map_age_basis: str = "receipt_time"
    source_age_ms: float | None = None
    left_source_age_ms: float | None = None


def safety_diagnostic_values(
    decision: SafetyDecision,
    config: SafetyConfig,
    map_age_ms: float,
    processing_ms: float,
    *,
    freshness_mode: str = NAV2_LIVE,
    map_age_basis: str = "receipt_time",
    source_age_ms: float | None = None,
    left_source_age_ms: float | None = None,
) -> list[KeyValue]:
    """Build stable, machine-readable evidence for one decision."""

    return [
        KeyValue(key="map_age_ms", value="%.3f" % map_age_ms),
        KeyValue(key="freshness_mode", value=freshness_mode),
        KeyValue(key="map_age_basis", value=map_age_basis),
        KeyValue(
            key="source_age_ms",
            value=(
                "none" if source_age_ms is None else "%.3f" % source_age_ms
            ),
        ),
        KeyValue(
            key="left_source_age_ms",
            value=(
                "none"
                if left_source_age_ms is None
                else "%.3f" % left_source_age_ms
            ),
        ),
        KeyValue(key="processing_ms", value="%.3f" % processing_ms),
        KeyValue(key="enable_motion", value=str(config.enable_motion)),
        KeyValue(
            key="geometry_calibrated",
            value=str(config.geometry_calibrated),
        ),
        KeyValue(key="min_steering", value="%.3f" % config.min_steering),
        KeyValue(key="max_steering", value="%.3f" % config.max_steering),
        KeyValue(
            key="turn_clearance_radius_m",
            value="%.3f" % config.turn_clearance_radius_m,
        ),
        KeyValue(
            key="clear_turn_limit",
            value="%.3f" % config.clear_turn_limit,
        ),
        KeyValue(
            key="slow_turn_limit",
            value="%.3f" % config.slow_turn_limit,
        ),
        KeyValue(
            key="turn_longitudinal_limit",
            value="%.3f" % config.turn_longitudinal_limit,
        ),
        KeyValue(
            key="nearest_path_distance_m",
            value=(
                "none"
                if decision.nearest_path_distance_m is None
                else "%.3f" % decision.nearest_path_distance_m
            ),
        ),
        KeyValue(
            key="maximum_path_cost",
            value=(
                "none"
                if decision.maximum_path_cost is None
                else str(decision.maximum_path_cost)
            ),
        ),
        KeyValue(
            key="nearest_slow_cost_distance_m",
            value=(
                "none"
                if decision.nearest_slow_cost_distance_m is None
                else "%.3f" % decision.nearest_slow_cost_distance_m
            ),
        ),
        KeyValue(
            key="nearest_stop_cost_distance_m",
            value=(
                "none"
                if decision.nearest_stop_cost_distance_m is None
                else "%.3f" % decision.nearest_stop_cost_distance_m
            ),
        ),
        KeyValue(
            key="slow_cost_threshold",
            value=str(config.slow_cost_threshold),
        ),
        KeyValue(
            key="stop_cost_threshold",
            value=str(config.stop_cost_threshold),
        ),
        KeyValue(
            key="path_cost_valid",
            value=str(decision.path_cost_valid),
        ),
    ]


def build_safety_diagnostic_status(
    snapshot: SafetyDiagnosticSnapshot,
) -> DiagnosticStatus:
    """Build one complete supervisor DiagnosticStatus message."""

    status = DiagnosticStatus()
    status.name = "wheelchair_shared_control/safety_supervisor"
    status.hardware_id = "jetson"
    status.level = (
        DiagnosticStatus.OK
        if snapshot.decision.decision == CLEAR
        else DiagnosticStatus.WARN
    )
    status.message = snapshot.decision.reason
    status.values = safety_diagnostic_values(
        snapshot.decision,
        snapshot.config,
        snapshot.map_age_ms,
        snapshot.processing_ms,
        freshness_mode=snapshot.freshness_mode,
        map_age_basis=snapshot.map_age_basis,
        source_age_ms=snapshot.source_age_ms,
        left_source_age_ms=snapshot.left_source_age_ms,
    )
    return status


def format_decision_transition(decision: SafetyDecision) -> str:
    """Format the existing log line for a changed safety decision."""

    return "Safety decision changed: %s (nearest=%s max_cost=%s)" % (
        decision.reason,
        (
            "none"
            if decision.nearest_path_distance_m is None
            else "%.3f m" % decision.nearest_path_distance_m
        ),
        (
            "none"
            if decision.maximum_path_cost is None
            else str(decision.maximum_path_cost)
        ),
    )


__all__ = [
    "SafetyDiagnosticSnapshot",
    "build_safety_diagnostic_status",
    "format_decision_transition",
    "safety_diagnostic_values",
]
