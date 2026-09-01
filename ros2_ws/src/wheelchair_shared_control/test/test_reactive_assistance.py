import time

import numpy as np

from wheelchair_shared_control.models import (
    SLOW,
    OperatorIntentData,
    SafetyConfig,
    SafetyDecision,
    weighted_costmap_from_grid,
)
from wheelchair_shared_control.operator_intent import (
    FORWARD,
    FORWARD_LEFT,
    FORWARD_RIGHT,
)
from wheelchair_shared_control.reactive_assistance import (
    ReactiveCandidate,
    ReactiveConfig,
    ReactiveConfirmation,
    ReactiveSelection,
    apply_reactive_steering,
    available_reactive_authority,
    generate_candidate_steering,
    resolve_reactive_decision,
    select_reactive_steering,
)
from wheelchair_shared_control.safety_policy import evaluate_safety


SAFETY = SafetyConfig(
    enable_motion=True,
    geometry_calibrated=True,
    stop_distance_m=0.7,
    slow_distance_m=1.2,
)
REACTIVE = ReactiveConfig()


def _costmap(bands):
    resolution = 0.02
    width = 70
    height = 60
    origin_y = -0.6
    values = np.zeros((height, width), dtype=np.int16)
    xs = (np.arange(width) + 0.5) * resolution
    ys = origin_y + (np.arange(height) + 0.5) * resolution
    for x_min, x_max, y_min, y_max, cost in bands:
        mask = (
            (xs[None, :] >= x_min)
            & (xs[None, :] <= x_max)
            & (ys[:, None] >= y_min)
            & (ys[:, None] <= y_max)
        )
        values[mask] = cost
    return weighted_costmap_from_grid(
        values.ravel(),
        frame_id="base_link",
        width=width,
        height=height,
        resolution_m=resolution,
        origin_x_m=0.0,
        origin_y_m=origin_y,
        origin_orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
    )


def _direct(costmap, lateral=0.0, intent_class=FORWARD):
    return evaluate_safety(
        OperatorIntentData(
            "session", 1, lateral, 1.0, intent_class, True
        ),
        costmap,
        SAFETY,
    )


def test_candidate_fans_include_exact_boundaries_and_do_not_cross_zero():
    straight = generate_candidate_steering(
        0.0, FORWARD, 0.15, REACTIVE, SAFETY
    )
    left = generate_candidate_steering(
        0.30, FORWARD_LEFT, 0.15, REACTIVE, SAFETY
    )
    right = generate_candidate_steering(
        -0.30, FORWARD_RIGHT, 0.15, REACTIVE, SAFETY
    )

    assert straight[0] == -0.15
    assert straight[-1] == 0.15
    assert 0.0 in straight
    assert left[0] == 0.15 and left[-1] == 0.30
    assert right[0] == -0.30 and right[-1] == -0.15
    assert all(value >= 0.0 for value in left)
    assert all(value <= 0.0 for value in right)


def test_slow_to_clear_selects_left_deterministically_and_keeps_direct_cap():
    costmap = _costmap(((0.74, 0.86, -0.021, 0.021, 50),))
    direct = _direct(costmap)
    selection = select_reactive_steering(
        costmap=costmap,
        requested_steering=0.0,
        intent_class=FORWARD,
        authority=0.15,
        direct=direct,
        config=REACTIVE,
        safety_config=SAFETY,
    )

    assert direct.decision == SLOW
    assert selection.valid
    assert selection.selected_steering > 0.0
    applied = apply_reactive_steering(direct, selection.selected_steering)
    assert applied.reason == "nav2_cost_slow"
    assert applied.permitted_forward == direct.permitted_forward
    assert applied.maximum_path_cost == direct.maximum_path_cost
    assert applied.permitted_steering == selection.selected_steering


def test_slow_to_slow_requires_configured_cost_improvement():
    improved = _costmap(
        (
            (0.74, 0.86, -0.09, 0.09, 40),
            (0.74, 0.86, -0.021, 0.021, 50),
        )
    )
    insufficient = _costmap(
        (
            (0.74, 0.86, -0.09, 0.09, 48),
            (0.74, 0.86, -0.021, 0.021, 50),
        )
    )
    accepted = select_reactive_steering(
        costmap=improved,
        requested_steering=0.0,
        intent_class=FORWARD,
        authority=0.15,
        direct=_direct(improved),
        config=REACTIVE,
        safety_config=SAFETY,
    )
    rejected = select_reactive_steering(
        costmap=insufficient,
        requested_steering=0.0,
        intent_class=FORWARD,
        authority=0.15,
        direct=_direct(insufficient),
        config=REACTIVE,
        safety_config=SAFETY,
    )

    assert accepted.valid
    assert accepted.selected.decision == SLOW
    assert accepted.cost_improvement >= 5
    assert not rejected.valid
    assert rejected.status == "insufficient_improvement"


def test_stop_cost_anywhere_on_candidate_is_rejected():
    costmap = _costmap(
        (
            (0.74, 0.86, -0.021, 0.021, 50),
            (0.74, 0.86, 0.03, 0.06, 99),
        )
    )
    selection = select_reactive_steering(
        costmap=costmap,
        requested_steering=0.0,
        intent_class=FORWARD,
        authority=0.15,
        direct=_direct(costmap),
        config=REACTIVE,
        safety_config=SAFETY,
    )

    assert any(
        candidate.rejection_reason == "stop_cost_on_reactive_path"
        for candidate in selection.candidates
    )
    assert selection.selected_steering < 0.0


def test_clear_stop_reverse_and_hard_turn_decisions_are_ineligible():
    empty = _costmap(())
    direct_clear = _direct(empty)
    for direct, intent_class in (
        (direct_clear, FORWARD),
        (SafetyDecision(0, 0.0, 0.0, "nav2_cost_stop"), FORWARD),
        (SafetyDecision(SLOW, 0.5, 0.0, "reverse_unmonitored_slow"), 6),
        (SafetyDecision(SLOW, 0.1, 0.6, "nav2_turn_cost_slow"), 4),
    ):
        selection = select_reactive_steering(
            costmap=empty,
            requested_steering=0.0,
            intent_class=intent_class,
            authority=0.15,
            direct=direct,
            config=REACTIVE,
            safety_config=SAFETY,
        )
        assert not selection.valid


def test_confirmation_requires_two_cycles_and_resets_on_semantic_change():
    source = ReactiveCandidate(0.0, SLOW, 50, 100, 0.8, True)
    selected = ReactiveCandidate(0.10, 2, 0, 0, None, True)
    selection = ReactiveSelection(
        True,
        "candidate_selected",
        0.0,
        0.10,
        source,
        (source, selected),
        50,
    )
    confirmation = ReactiveConfirmation(REACTIVE)

    first = confirmation.update(
        selection,
        session_id="session",
        intent_class=FORWARD,
        source_steering=0.0,
    )
    second = confirmation.update(
        selection,
        session_id="session",
        intent_class=FORWARD,
        source_steering=0.0,
    )
    changed = confirmation.update(
        selection,
        session_id="new-session",
        intent_class=FORWARD,
        source_steering=0.0,
    )

    assert not first.confirmed and first.count == 1
    assert second.confirmed and second.steering == 0.10
    assert not changed.confirmed and changed.count == 1
    confirmation.reset()
    assert confirmation.count == 0


def test_shadow_uses_system_authority_but_never_changes_envelope():
    direct = SafetyDecision(
        SLOW,
        0.6,
        0.0,
        "nav2_cost_slow",
        maximum_path_cost=50,
        path_cost_valid=True,
    )

    assert available_reactive_authority("shadow", 0.0, REACTIVE) == 0.15
    assert available_reactive_authority("enforce", 0.0, REACTIVE) == 0.0
    assert available_reactive_authority("enforce", 0.08, REACTIVE) == 0.08
    assert available_reactive_authority("enforce", 0.50, REACTIVE) == 0.15
    assert resolve_reactive_decision("shadow", direct, 0.10) is direct
    enforced = resolve_reactive_decision("enforce", direct, 0.10)
    assert enforced.permitted_steering == 0.10
    assert enforced.permitted_forward == 0.6
    assert enforced.reason == "nav2_cost_slow"


def test_selector_latency_budget_on_representative_jetson_grid():
    costmap = _costmap(((0.74, 0.86, -0.021, 0.021, 50),))
    direct = _direct(costmap)
    timings = []
    for _ in range(100):
        started = time.perf_counter()
        select_reactive_steering(
            costmap=costmap,
            requested_steering=0.0,
            intent_class=FORWARD,
            authority=0.15,
            direct=direct,
            config=REACTIVE,
            safety_config=SAFETY,
        )
        timings.append((time.perf_counter() - started) * 1000.0)

    assert np.percentile(timings, 95) < 10.0
    assert max(timings) < 25.0
