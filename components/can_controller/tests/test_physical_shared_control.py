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
        self.assertEqual(link.calls[-1], (0, 80, True))

    def test_steering_latches_stop_until_forward_axis_returns_neutral(self):
        link = FakeSafetyLink(safe_output=(0, 20))
        control = StraightPhysicalJsmControl(link, mode="enforce", neutral_deadzone=5)

        self.assertEqual(control.transform(sample(10, 80)), (0, 0))
        self.assertEqual(control.last_result.reason, "straight_only_steering")
        self.assertEqual(control.transform(sample(0, 80)), (0, 0))
        self.assertEqual(control.last_result.reason, "local_stop_latched")
        self.assertEqual(control.transform(sample(0, 0)), (0, 0))
        self.assertFalse(control.last_result.local_stop_latched)
        self.assertEqual(control.transform(sample(0, 80)), (0, 20))

    def test_reverse_is_locally_stopped_and_latched(self):
        link = FakeSafetyLink()
        control = StraightPhysicalJsmControl(link, mode="enforce")

        self.assertEqual(control.transform(sample(0, -20)), (0, 0))
        self.assertEqual(control.last_result.reason, "reverse_disabled")
        self.assertTrue(control.last_result.local_stop_latched)
