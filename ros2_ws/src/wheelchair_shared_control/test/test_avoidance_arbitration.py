import math

import pytest

from wheelchair_shared_control.avoidance_arbitration import (
    bound_current_assist,
    candidate_is_no_worse,
    suggestion_age_is_fresh,
    suggestion_compatibility_reason,
)
from wheelchair_shared_control.operator_intent import (
    FORWARD,
    FORWARD_LEFT,
    FORWARD_RIGHT,
    REVERSE,
)


def _bound(requested, intent_class, planned, authority=0.15):
    return bound_current_assist(
        requested=requested,
        intent_class=intent_class,
        planned=planned,
        authority=authority,
        system_maximum=0.15,
        minimum_steering=-0.577350269,
        maximum_steering=0.577350269,
    )


def test_current_authority_is_rechecked_for_every_forward_class():
    assert _bound(0.0, FORWARD, 0.4) == pytest.approx(0.15)
    assert _bound(0.0, FORWARD, -0.4) == pytest.approx(-0.15)
    assert _bound(0.30, FORWARD_LEFT, -0.4) == pytest.approx(0.15)
    assert _bound(-0.30, FORWARD_RIGHT, 0.4) == pytest.approx(-0.15)
    assert _bound(0.0, REVERSE, 0.1) is None
    assert _bound(0.0, FORWARD, 0.1, authority=0.0) is None


@pytest.mark.parametrize(
    "changes, expected",
    [
        ({"suggestion_session": "other"}, "suggestion_intent_mismatch"),
        ({"suggestion_sequence": 12}, "suggestion_intent_mismatch"),
        ({"current_intent_class": REVERSE}, "suggestion_intent_mismatch"),
        ({"source_steering": 0.051}, "source_steering_mismatch"),
        ({"suggested_steering": math.nan}, "invalid_suggestion"),
        ({"valid": False}, "invalid_suggestion"),
    ],
)
def test_mismatched_or_invalid_suggestion_fails_closed(changes, expected):
    arguments = dict(
        current_session="session",
        current_sequence=11,
        current_intent_class=FORWARD,
        current_steering=0.0,
        suggestion_session="session",
        suggestion_sequence=10,
        suggestion_intent_class=FORWARD,
        source_steering=0.0,
        suggested_steering=0.1,
        valid=True,
        source_tolerance=0.05,
    )
    arguments.update(changes)
    assert suggestion_compatibility_reason(**arguments) == expected


def test_matching_freshness_inputs_are_compatible():
    assert suggestion_compatibility_reason(
        current_session="session",
        current_sequence=11,
        current_intent_class=FORWARD_LEFT,
        current_steering=0.2,
        suggestion_session="session",
        suggestion_sequence=10,
        suggestion_intent_class=FORWARD_LEFT,
        source_steering=0.16,
        suggested_steering=0.1,
        valid=True,
        source_tolerance=0.05,
    ) is None


def test_suggestion_age_expires_fail_closed_at_quarter_second():
    assert suggestion_age_is_fresh(0.0, 0.25)
    assert suggestion_age_is_fresh(0.25, 0.25)
    assert not suggestion_age_is_fresh(0.251, 0.25)
    assert not suggestion_age_is_fresh(-0.001, 0.25)


def test_candidate_must_be_non_stop_and_no_worse_than_direct():
    assert candidate_is_no_worse(0, 1)
    assert candidate_is_no_worse(1, 1)
    assert candidate_is_no_worse(1, 2)
    assert not candidate_is_no_worse(2, 1)
    assert not candidate_is_no_worse(0, 0)
