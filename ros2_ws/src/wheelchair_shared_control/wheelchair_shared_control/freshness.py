"""Pure freshness policy for supervisor intent and map inputs."""

from __future__ import annotations

from dataclasses import dataclass


NAV2_LIVE = "nav2_live"
LEGACY_MAP_STAMP = "legacy_map_stamp"
FRESHNESS_MODES = (NAV2_LIVE, LEGACY_MAP_STAMP)


@dataclass(frozen=True)
class FreshnessPolicy:
    """Timeouts and clock policy for one supervisor configuration."""

    mode: str = NAV2_LIVE
    max_intent_age_s: float = 0.20
    max_map_age_s: float = 0.50
    max_source_age_s: float = 0.50
    max_future_source_offset_s: float = 0.10


@dataclass(frozen=True)
class FreshnessInputs:
    """Timestamps and presence state observed for one decision cycle."""

    now_ros_ns: int
    now_monotonic_ns: int
    intent_stamp_ns: int | None
    map_available: bool
    map_stamp_ns: int
    map_received_monotonic_ns: int
    source_stamp_ns: int | None


@dataclass(frozen=True)
class FreshnessStatus:
    """Ages and any fail-closed reason for one supervisor cycle."""

    intent_age_s: float = 0.0
    map_age_s: float = 0.0
    source_age_s: float | None = None
    failure_reason: str | None = None
    map_age_basis: str = "receipt_time"


def validate_freshness_policy(policy: FreshnessPolicy) -> None:
    """Validate the freshness settings checked at node startup today."""

    if policy.mode not in FRESHNESS_MODES:
        raise ValueError(
            "freshness_mode must be one of %s" % ", ".join(FRESHNESS_MODES)
        )
    if policy.max_source_age_s <= 0.0:
        raise ValueError("max_source_age_s must be positive")
    if policy.max_future_source_offset_s < 0.0:
        raise ValueError("max_future_source_offset_s must be non-negative")


def evaluate_freshness(
    inputs: FreshnessInputs,
    policy: FreshnessPolicy,
) -> FreshnessStatus:
    """Evaluate intent, map, and source freshness in fail-closed order."""

    basis = (
        "receipt_time" if policy.mode == NAV2_LIVE else "map_header_stamp"
    )
    if policy.mode not in FRESHNESS_MODES:
        raise ValueError("unsupported freshness_mode: %s" % policy.mode)

    if inputs.intent_stamp_ns is None:
        return FreshnessStatus(
            failure_reason="missing_intent",
            map_age_basis=basis,
        )
    intent_age_s = (inputs.now_ros_ns - inputs.intent_stamp_ns) / 1e9
    if inputs.intent_stamp_ns <= 0 or intent_age_s < 0.0:
        return FreshnessStatus(
            intent_age_s=intent_age_s,
            failure_reason="invalid_intent_timestamp",
            map_age_basis=basis,
        )
    if intent_age_s > policy.max_intent_age_s:
        return FreshnessStatus(
            intent_age_s=intent_age_s,
            failure_reason="stale_intent",
            map_age_basis=basis,
        )
    if not inputs.map_available:
        return FreshnessStatus(
            intent_age_s=intent_age_s,
            failure_reason="missing_map",
            map_age_basis=basis,
        )

    if policy.mode == LEGACY_MAP_STAMP:
        if inputs.map_stamp_ns <= 0:
            return FreshnessStatus(
                intent_age_s=intent_age_s,
                failure_reason="invalid_map_timestamp",
                map_age_basis=basis,
            )
        map_age_s = (inputs.now_ros_ns - inputs.map_stamp_ns) / 1e9
        if map_age_s < 0.0:
            return FreshnessStatus(
                intent_age_s=intent_age_s,
                map_age_s=map_age_s,
                failure_reason="invalid_map_age",
                map_age_basis=basis,
            )
        if map_age_s > policy.max_map_age_s:
            return FreshnessStatus(
                intent_age_s=intent_age_s,
                map_age_s=map_age_s,
                failure_reason="stale_map",
                map_age_basis=basis,
            )
        return FreshnessStatus(
            intent_age_s=intent_age_s,
            map_age_s=map_age_s,
            map_age_basis=basis,
        )

    if inputs.map_received_monotonic_ns <= 0:
        return FreshnessStatus(
            intent_age_s=intent_age_s,
            failure_reason="missing_map_receipt",
            map_age_basis=basis,
        )
    map_age_s = (
        inputs.now_monotonic_ns - inputs.map_received_monotonic_ns
    ) / 1e9
    if map_age_s < 0.0:
        return FreshnessStatus(
            intent_age_s=intent_age_s,
            failure_reason="invalid_map_receipt_time",
            map_age_basis=basis,
        )
    if inputs.source_stamp_ns is None:
        return FreshnessStatus(
            intent_age_s=intent_age_s,
            map_age_s=map_age_s,
            failure_reason="missing_source_heartbeat",
            map_age_basis=basis,
        )
    if inputs.source_stamp_ns <= 0:
        return FreshnessStatus(
            intent_age_s=intent_age_s,
            map_age_s=map_age_s,
            failure_reason="invalid_source_timestamp",
            map_age_basis=basis,
        )
    source_age_s = (inputs.now_ros_ns - inputs.source_stamp_ns) / 1e9
    if source_age_s < -policy.max_future_source_offset_s:
        return FreshnessStatus(
            intent_age_s=intent_age_s,
            map_age_s=map_age_s,
            source_age_s=source_age_s,
            failure_reason="future_source_timestamp",
            map_age_basis=basis,
        )
    source_age_s = max(0.0, source_age_s)
    if source_age_s > policy.max_source_age_s:
        return FreshnessStatus(
            intent_age_s=intent_age_s,
            map_age_s=map_age_s,
            source_age_s=source_age_s,
            failure_reason="stale_source",
            map_age_basis=basis,
        )
    if map_age_s > policy.max_map_age_s:
        return FreshnessStatus(
            intent_age_s=intent_age_s,
            map_age_s=map_age_s,
            source_age_s=source_age_s,
            failure_reason="stale_map",
            map_age_basis=basis,
        )
    return FreshnessStatus(
        intent_age_s=intent_age_s,
        map_age_s=map_age_s,
        source_age_s=source_age_s,
        map_age_basis=basis,
    )


__all__ = [
    "FRESHNESS_MODES",
    "LEGACY_MAP_STAMP",
    "NAV2_LIVE",
    "FreshnessInputs",
    "FreshnessPolicy",
    "FreshnessStatus",
    "evaluate_freshness",
    "validate_freshness_policy",
]
