from pathlib import Path
import sys
import unittest


COMPONENT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(COMPONENT / "src"))

from wheelchair_teleop.jsm_observer import JsmSample
from wheelchair_teleop.physical_shared_control import StraightPhysicalJsmControl


class FakeSafetyLink:
    enabled = True

    def __init__(self, safe_output=(0, 20), reason="clear", decision=2):
        self.safe_output = safe_output
        self.reason = reason
        self.decision = decision
        self.calls = []

    def apply(self, x_pos, y_pos, deadman):
        self.calls.append((x_pos, y_pos, deadman))
        return self.safe_output

    def get_status(self):
        return {
            "reason": self.reason,
            "latest_decision": self.decision,
            "map_age_ms": 20.0,
            "round_trip_ms": 5.0,
            "envelope_age_ms": 1.0,
        }


def sample(x_raw, y_raw):
    return JsmSample(
        1.0, 2.0, 0.01, 0x02000200, x_raw, y_raw,
        -x_raw / 100.0, max(0.0, y_raw / 100.0),
        max(0.0, -y_raw / 100.0)
    )


class StraightPhysicalJsmControlTests(unittest.TestCase):
    def test_shadow_reports_safe_output_but_preserves_physical_axes(self):
        link = FakeSafetyLink(safe_output=(0, 15), reason="obstacle_slow", decision=1)
        control = StraightPhysicalJsmControl(link, mode="shadow")

        forwarded = control.transform(sample(2, 80))

        self.assertEqual(forwarded, (2, 80))
        self.assertEqual(
            (control.last_result.would_output_x, control.last_result.would_output_y),
            (0, 15),
        )

    def test_enforce_forwards_only_supervised_straight_command(self):
        link = FakeSafetyLink(safe_output=(0, 20))
        control = StraightPhysicalJsmControl(link, mode="enforce", neutral_deadzone=5)

        self.assertEqual(control.transform(sample(4, 80)), (0, 20))
        self.assertEqual(link.calls[-1], (4, 80, True))

    def test_hard_turn_is_supervised_without_local_latch(self):
        link = FakeSafetyLink(safe_output=(60, 15))
        control = StraightPhysicalJsmControl(link, mode="enforce", neutral_deadzone=5)

        self.assertEqual(control.transform(sample(90, 20)), (60, 15))
        self.assertEqual(link.calls[-1], (90, 20, True))
        self.assertEqual(control.last_result.intent_label, "right_turn")
        self.assertFalse(control.last_result.local_stop_latched)

    def test_forward_correction_is_supervised_without_local_latch(self):
        link = FakeSafetyLink(safe_output=(-3, 20))
        control = StraightPhysicalJsmControl(link, mode="enforce")

        self.assertEqual(control.transform(sample(-14, 99)), (-3, 20))
        self.assertEqual(link.calls[-1], (-14, 99, True))
        self.assertEqual(control.last_result.intent_label, "forward_left")
        self.assertFalse(control.last_result.local_stop_latched)

    def test_reverse_cone_is_supervised_without_local_latch(self):
        link = FakeSafetyLink(
            safe_output=(13, -65),
            reason="reverse_unmonitored_slow",
            decision=1,
        )
        control = StraightPhysicalJsmControl(link, mode="enforce")

        self.assertEqual(control.transform(sample(20, -100)), (13, -65))
        self.assertEqual(link.calls[-1], (20, -100, True))
        self.assertEqual(
            control.last_result.reason, "reverse_unmonitored_slow"
        )
        self.assertFalse(control.last_result.local_stop_latched)

    def test_hard_turn_with_reverse_adjustment_is_supervised(self):
        link = FakeSafetyLink(safe_output=(60, -15))
        control = StraightPhysicalJsmControl(link, mode="enforce")

        self.assertEqual(control.transform(sample(100, -20)), (60, -15))
        self.assertEqual(link.calls[-1], (100, -20, True))
        self.assertFalse(control.last_result.local_stop_latched)

    def test_pure_lateral_input_is_not_treated_as_release(self):
        link = FakeSafetyLink(safe_output=(60, 0))
        control = StraightPhysicalJsmControl(link, mode="enforce")

        self.assertEqual(control.transform(sample(100, 0)), (60, 0))
        self.assertEqual(control.last_result.intent_label, "right_turn")
        self.assertEqual(link.calls[-1], (100, 0, True))
