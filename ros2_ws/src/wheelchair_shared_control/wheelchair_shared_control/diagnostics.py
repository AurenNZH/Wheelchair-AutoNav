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
    reactive_mode: str = "disabled"
    reactive_status: str = "disabled"
    intent_sequence: int = 0
    requested_steering: float | None = None
    selected_steering: float | None = None
    advertised_authority: float = 0.0
    applied_authority: float = 0.0
    candidate_count: int = 0
    rejected_candidate_count: int = 0
    requested_maximum_cost: int | None = None
    requested_accumulated_cost: int | None = None
    selected_maximum_cost: int | None = None
    selected_accumulated_cost: int | None = None
    selected_first_inflated_distance_m: float | None = None
    cost_improvement: int | None = None
    confirmation_count: int = 0
    reactive_processing_ms: float = 0.0
    reactive_suggestions: int = 0
    reactive_enforcements: int = 0


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
    reactive_mode: str = "disabled",
    reactive_status: str = "disabled",
    intent_sequence: int = 0,
    requested_steering: float | None = None,
    selected_steering: float | None = None,
    advertised_authority: float = 0.0,
    applied_authority: float = 0.0,
    candidate_count: int = 0,
    rejected_candidate_count: int = 0,
    requested_maximum_cost: int | None = None,
    requested_accumulated_cost: int | None = None,
    selected_maximum_cost: int | None = None,
    selected_accumulated_cost: int | None = None,
    selected_first_inflated_distance_m: float | None = None,
    cost_improvement: int | None = None,
    confirmation_count: int = 0,
    reactive_processing_ms: float = 0.0,
    reactive_suggestions: int = 0,
    reactive_enforcements: int = 0,
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
        KeyValue(key="reactive_assistance_mode", value=reactive_mode),
        KeyValue(key="reactive_status", value=reactive_status),
        KeyValue(key="intent_sequence", value=str(intent_sequence)),
        KeyValue(
            key="requested_steering",
            value=_optional_float(requested_steering),
        ),
        KeyValue(
            key="selected_steering",
            value=_optional_float(selected_steering),
        ),
        KeyValue(
            key="advertised_authority",
            value="%.3f" % advertised_authority,
        ),
        KeyValue(
            key="applied_authority", value="%.3f" % applied_authority
        ),
        KeyValue(key="candidate_count", value=str(candidate_count)),
        KeyValue(
            key="rejected_candidate_count",
            value=str(rejected_candidate_count),
        ),
        KeyValue(
            key="requested_maximum_cost",
            value=_optional_int(requested_maximum_cost),
        ),
        KeyValue(
            key="requested_accumulated_cost",
            value=_optional_int(requested_accumulated_cost),
        ),
        KeyValue(
            key="selected_maximum_cost",
            value=_optional_int(selected_maximum_cost),
        ),
        KeyValue(
            key="selected_accumulated_cost",
            value=_optional_int(selected_accumulated_cost),
        ),
        KeyValue(
            key="selected_first_inflated_distance_m",
            value=_optional_float(selected_first_inflated_distance_m),
        ),
        KeyValue(
            key="cost_improvement", value=_optional_int(cost_improvement)
        ),
        KeyValue(
            key="confirmation_count", value=str(confirmation_count)
        ),
        KeyValue(
            key="reactive_processing_ms",
            value="%.3f" % reactive_processing_ms,
        ),
        KeyValue(
            key="reactive_suggestions", value=str(reactive_suggestions)
        ),
        KeyValue(
            key="reactive_enforcements", value=str(reactive_enforcements)
        ),
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
        reactive_mode=snapshot.reactive_mode,
        reactive_status=snapshot.reactive_status,
        intent_sequence=snapshot.intent_sequence,
        requested_steering=snapshot.requested_steering,
        selected_steering=snapshot.selected_steering,
        advertised_authority=snapshot.advertised_authority,
        applied_authority=snapshot.applied_authority,
        candidate_count=snapshot.candidate_count,
        rejected_candidate_count=snapshot.rejected_candidate_count,
        requested_maximum_cost=snapshot.requested_maximum_cost,
        requested_accumulated_cost=(
            snapshot.requested_accumulated_cost
        ),
        selected_maximum_cost=snapshot.selected_maximum_cost,
        selected_accumulated_cost=snapshot.selected_accumulated_cost,
        selected_first_inflated_distance_m=(
            snapshot.selected_first_inflated_distance_m
        ),
        cost_improvement=snapshot.cost_improvement,
        confirmation_count=snapshot.confirmation_count,
        reactive_processing_ms=snapshot.reactive_processing_ms,
        reactive_suggestions=snapshot.reactive_suggestions,
        reactive_enforcements=snapshot.reactive_enforcements,
    )
    return status


def _optional_float(value: float | None) -> str:
    return "none" if value is None else "%.3f" % value


def _optional_int(value: int | None) -> str:
    return "none" if value is None else str(value)


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
