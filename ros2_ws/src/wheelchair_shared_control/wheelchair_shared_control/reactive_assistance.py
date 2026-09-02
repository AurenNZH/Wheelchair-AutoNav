"""Low-latency local steering selection for SLOW forward trajectories."""

from __future__ import annotations

from dataclasses import dataclass
import math

from wheelchair_shared_control.models import (
    CLEAR,
    SLOW,
    STOP,
    SafetyConfig,
    SafetyDecision,
    WeightedCostmap,
)
from wheelchair_shared_control.operator_intent import (
    FORWARD,
    FORWARD_CLASSES,
    FORWARD_LEFT,
    FORWARD_RIGHT,
)
from wheelchair_shared_control.trajectory import (
    PathCostSummary,
    individual_path_costs_batch,
)


@dataclass(frozen=True)
class ReactiveConfig:
    horizon_m: float = 1.20
    path_sample_step_m: float = 0.05
    steering_step: float = 0.05
    minimum_correction: float = 0.02
    minimum_cost_improvement: int = 5
    confirmation_cycles: int = 2
    intent_change_tolerance: float = 0.05
    maximum_steering_assist: float = 0.577350269


@dataclass(frozen=True)
class ReactiveCandidate:
    steering: float
    decision: int
    maximum_cost: int | None
    accumulated_cost: int
    first_inflated_distance_m: float | None
    valid: bool
    rejection_reason: str = ""


@dataclass(frozen=True)
class ReactiveSelection:
    valid: bool
    status: str
    requested_steering: float
    selected_steering: float | None
    requested: ReactiveCandidate
    candidates: tuple[ReactiveCandidate, ...]
    cost_improvement: int | None = None

    @property
    def candidate_count(self) -> int:
        return len(self.candidates)

    @property
    def rejected_count(self) -> int:
        return sum(not candidate.valid for candidate in self.candidates)

    @property
    def selected(self) -> ReactiveCandidate | None:
        if self.selected_steering is None:
            return None
        return min(
            self.candidates,
            key=lambda candidate: abs(
                candidate.steering - float(self.selected_steering)
            ),
        )


@dataclass(frozen=True)
class ConfirmationResult:
    steering: float | None
    count: int
    confirmed: bool
    status: str


def validate_reactive_config(config: ReactiveConfig) -> None:
    positive = (
        config.horizon_m,
        config.path_sample_step_m,
        config.steering_step,
        config.minimum_correction,
        config.intent_change_tolerance,
    )
    if not all(math.isfinite(value) and value > 0.0 for value in positive):
        raise ValueError("reactive distances and limits must be positive")
    if (
        not math.isfinite(config.maximum_steering_assist)
        or not 0.0 <= config.maximum_steering_assist <= 1.0
    ):
        raise ValueError("maximum steering assist must be in [0, 1]")
    if config.minimum_cost_improvement < 0:
        raise ValueError("minimum cost improvement must be non-negative")
    if config.confirmation_cycles < 1:
        raise ValueError("confirmation cycles must be positive")


def available_reactive_authority(
    mode: str,
    advertised_authority: float,
    config: ReactiveConfig,
) -> float | None:
    """Resolve shadow/system and enforce/packet steering authority."""

    if mode not in ("disabled", "shadow", "enforce"):
        raise ValueError("invalid reactive assistance mode")
    advertised = float(advertised_authority)
    if not math.isfinite(advertised):
        return None
    if mode == "disabled":
        return 0.0
    if mode == "shadow":
        return config.maximum_steering_assist
    return min(max(0.0, advertised), config.maximum_steering_assist)


def generate_candidate_steering(
    requested: float,
    intent_class: int,
    authority: float,
    config: ReactiveConfig,
    safety_config: SafetyConfig,
) -> tuple[float, ...]:
    """Return the bounded fan, including its source and exact endpoints."""

    validate_reactive_config(config)
    requested = float(requested)
    authority = max(0.0, min(float(authority), config.maximum_steering_assist))
    if not math.isfinite(requested) or not math.isfinite(authority):
        return ()
    if intent_class == FORWARD:
        lower = max(safety_config.min_steering, requested - authority)
        upper = min(safety_config.max_steering, requested + authority)
    elif intent_class == FORWARD_LEFT:
        lower = max(0.0, requested - authority)
        upper = requested
    elif intent_class == FORWARD_RIGHT:
        lower = requested
        upper = min(0.0, requested + authority)
    else:
        return ()
    if lower > upper + 1e-9:
        return ()

    values = [lower, upper, requested]
    value = lower
    while value < upper - 1e-9:
        values.append(value)
        value += config.steering_step
    values.append(upper)
    return tuple(sorted({round(item, 9) for item in values}))


def select_reactive_steering(
    *,
    costmap: WeightedCostmap,
    requested_steering: float,
    intent_class: int,
    authority: float,
    direct: SafetyDecision,
    config: ReactiveConfig,
    safety_config: SafetyConfig,
    previous_side: int = 0,
) -> ReactiveSelection:
    """Rank individual arcs while preserving the direct SLOW decision."""

    if intent_class not in FORWARD_CLASSES:
        return _empty_selection(
            "ineligible_intent",
            _unscored_candidate(requested_steering),
        )
    if direct.decision != SLOW or direct.reason != "nav2_cost_slow":
        return _empty_selection(
            "direct_not_slow",
            _unscored_candidate(requested_steering),
        )

    steerings = generate_candidate_steering(
        requested_steering,
        intent_class,
        authority,
        config,
        safety_config,
    )
    summaries = individual_path_costs_batch(
        costmap,
        steerings,
        safety_config,
        horizon_m=config.horizon_m,
        sample_step_m=config.path_sample_step_m,
    )
    candidates = tuple(
        _candidate_from_summary(steering, summary)
        for steering, summary in zip(steerings, summaries)
    )
    if not candidates:
        return _empty_selection(
            "invalid_candidate_fan",
            _unscored_candidate(requested_steering),
        )
    requested = min(
        candidates,
        key=lambda candidate: abs(
            candidate.steering - requested_steering
        ),
    )
    alternatives = tuple(
        candidate
        for candidate in candidates
        if candidate.valid
        and abs(candidate.steering - requested_steering)
        >= config.minimum_correction
    )
    if not alternatives:
        return ReactiveSelection(
            False,
            "no_safe_alternative",
            requested_steering,
            None,
            requested,
            candidates,
        )

    selected = max(
        alternatives,
        key=lambda candidate: _candidate_rank(
            candidate, requested_steering, previous_side
        ),
    )
    direct_max = direct.maximum_path_cost
    improvement = (
        None
        if direct_max is None or selected.maximum_cost is None
        else int(direct_max) - int(selected.maximum_cost)
    )
    improves = selected.decision == CLEAR or (
        selected.decision == SLOW
        and improvement is not None
        and improvement >= config.minimum_cost_improvement
    )
    if not improves:
        return ReactiveSelection(
            False,
            "insufficient_improvement",
            requested_steering,
            selected.steering,
            requested,
            candidates,
            improvement,
        )
    return ReactiveSelection(
        True,
        "candidate_selected",
        requested_steering,
        selected.steering,
        requested,
        candidates,
        improvement,
    )


def apply_reactive_steering(
    direct: SafetyDecision, steering: float
) -> SafetyDecision:
    """Change steering only, retaining the direct SLOW cap and evidence."""

    if direct.decision != SLOW or direct.reason != "nav2_cost_slow":
        return direct
    return SafetyDecision(
        direct.decision,
        direct.permitted_forward,
        float(steering),
        direct.reason,
        direct.nearest_path_distance_m,
        direct.maximum_path_cost,
        direct.nearest_slow_cost_distance_m,
        direct.nearest_stop_cost_distance_m,
        direct.path_cost_valid,
    )


def resolve_reactive_decision(
    mode: str,
    direct: SafetyDecision,
    confirmed_steering: float | None,
) -> SafetyDecision:
    """Keep shadow observational and allow steering-only enforcement."""

    if mode not in ("disabled", "shadow", "enforce"):
        raise ValueError("invalid reactive assistance mode")
    if mode != "enforce" or confirmed_steering is None:
        return direct
    return apply_reactive_steering(direct, confirmed_steering)


class ReactiveConfirmation:
    """Require a stable correction side without adding a steering ramp."""

    def __init__(self, config: ReactiveConfig) -> None:
        validate_reactive_config(config)
        self._config = config
        self.reset()

    @property
    def preferred_side(self) -> int:
        return self._direction

    @property
    def count(self) -> int:
        return self._count

    def reset(self) -> None:
        self._session_id: str | None = None
        self._intent_class: int | None = None
        self._source_steering: float | None = None
        self._direction = 0
        self._count = 0

    def update(
        self,
        selection: ReactiveSelection,
        *,
        session_id: str,
        intent_class: int,
        source_steering: float,
    ) -> ConfirmationResult:
        self.prepare_context(
            session_id=session_id,
            intent_class=intent_class,
            source_steering=source_steering,
        )

        if not selection.valid or selection.selected_steering is None:
            self._direction = 0
            self._count = 0
            return ConfirmationResult(None, 0, False, selection.status)

        correction = selection.selected_steering - source_steering
        direction = 1 if correction > 0.0 else -1
        if direction == self._direction:
            self._count += 1
        else:
            self._direction = direction
            self._count = 1
        confirmed = self._count >= self._config.confirmation_cycles
        return ConfirmationResult(
            selection.selected_steering if confirmed else None,
            self._count,
            confirmed,
            "confirmed" if confirmed else "pending_confirmation",
        )

    def prepare_context(
        self,
        *,
        session_id: str,
        intent_class: int,
        source_steering: float,
    ) -> bool:
        """Reset hysteresis before ranking after a semantic intent change."""

        semantic_change = (
            self._session_id is not None
            and (
                session_id != self._session_id
                or intent_class != self._intent_class
                or self._source_steering is None
                or abs(source_steering - self._source_steering)
                > self._config.intent_change_tolerance
            )
        )
        if semantic_change:
            self.reset()
        self._session_id = session_id
        self._intent_class = intent_class
        self._source_steering = source_steering
        return semantic_change


def _empty_selection(
    status: str, requested: ReactiveCandidate
) -> ReactiveSelection:
    return ReactiveSelection(
        False,
        status,
        requested.steering,
        None,
        requested,
        (requested,),
    )


def _unscored_candidate(steering: float) -> ReactiveCandidate:
    return ReactiveCandidate(
        steering,
        STOP,
        None,
        0,
        None,
        False,
        "not_evaluated",
    )


def _candidate_from_summary(
    steering: float,
    summary: PathCostSummary,
) -> ReactiveCandidate:
    if not summary.valid:
        return ReactiveCandidate(
            steering,
            STOP,
            summary.maximum_cost,
            summary.accumulated_cost,
            summary.nearest_slow_distance_m,
            False,
            summary.failure_reason or "invalid_trajectory",
        )
    if summary.nearest_stop_distance_m is not None:
        return ReactiveCandidate(
            steering,
            STOP,
            summary.maximum_cost,
            summary.accumulated_cost,
            summary.nearest_slow_distance_m,
            False,
            "stop_cost_on_reactive_path",
        )
    decision = (
        SLOW if summary.nearest_slow_distance_m is not None else CLEAR
    )
    return ReactiveCandidate(
        steering,
        decision,
        summary.maximum_cost,
        summary.accumulated_cost,
        summary.nearest_slow_distance_m,
        True,
    )


def _candidate_rank(
    candidate: ReactiveCandidate,
    requested: float,
    previous_side: int,
) -> tuple[float, ...]:
    direction = 1 if candidate.steering - requested > 0.0 else -1
    first_inflated = (
        math.inf
        if candidate.first_inflated_distance_m is None
        else candidate.first_inflated_distance_m
    )
    return (
        1.0 if candidate.decision == CLEAR else 0.0,
        -float(candidate.maximum_cost or 0),
        first_inflated,
        -float(candidate.accumulated_cost),
        -abs(candidate.steering - requested),
        1.0 if previous_side and direction == previous_side else 0.0,
        1.0 if direction > 0 else 0.0,
    )


__all__ = [
    "ConfirmationResult",
    "ReactiveCandidate",
    "ReactiveConfig",
    "ReactiveConfirmation",
    "ReactiveSelection",
    "apply_reactive_steering",
    "available_reactive_authority",
    "generate_candidate_steering",
    "resolve_reactive_decision",
    "select_reactive_steering",
    "validate_reactive_config",
]
