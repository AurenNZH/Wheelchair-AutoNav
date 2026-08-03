import math
import unittest

from nav_msgs.msg import OccupancyGrid
from wheelchair_msgs.msg import SafetyEnvelope

from wheelchair_shared_control.envelope_monitor import (
    decision_name,
    envelope_signature,
    format_envelope,
)
from wheelchair_shared_control.intent_injector import (
    FORWARD,
    RELEASED,
    command_for_preset,
    motion_lease_expired,
    validate_injector_config,
)
from wheelchair_shared_control.replay_map_restamper import (
    restamped_grid,
    validate_topic_separation,
)


class IntentInjectorTests(unittest.TestCase):
    def test_released_and_forward_presets(self):
        released = command_for_preset(RELEASED, 0.5)
        forward = command_for_preset(FORWARD, 0.5)

        self.assertEqual((released.steering, released.forward), (0.0, 0.0))
        self.assertFalse(released.deadman)
        self.assertEqual((forward.steering, forward.forward), (0.0, 0.5))
        self.assertTrue(forward.deadman)

    def test_invalid_startup_values_are_rejected(self):
        common = {
            "command": RELEASED,
            "forward_request": 0.5,
            "publish_rate_hz": 20.0,
            "motion_timeout_s": 30.0,
        }
        invalid = (
            dict(common, command="slow"),
            dict(common, forward_request=0.0),
            dict(common, forward_request=1.01),
            dict(common, forward_request=math.nan),
            dict(common, publish_rate_hz=0.0),
            dict(common, motion_timeout_s=0.0),
        )
        for parameters in invalid:
            with self.subTest(parameters=parameters):
                with self.assertRaises(ValueError):
                    validate_injector_config(**parameters)

    def test_only_motion_commands_expire(self):
        self.assertFalse(motion_lease_expired(RELEASED, None, 100.0, 30.0))
        self.assertFalse(motion_lease_expired(FORWARD, 80.0, 100.0, 30.0))
        self.assertTrue(motion_lease_expired(FORWARD, 70.0, 100.0, 30.0))


class ReplayMapRestamperTests(unittest.TestCase):
    def test_only_header_timestamp_changes(self):
        original = OccupancyGrid()
        original.header.stamp.sec = 12
        original.header.frame_id = "base_link"
        original.info.resolution = 0.1
        original.info.width = 2
        original.info.height = 1
        original.info.origin.position.x = -1.0
        original.data = [0, 100]

        stamp = type(original.header.stamp)()
        stamp.sec = 99
        output = restamped_grid(original, stamp)

        self.assertEqual(original.header.stamp.sec, 12)
        self.assertEqual(output.header.stamp.sec, 99)
        self.assertEqual(output.header.frame_id, original.header.frame_id)
        self.assertEqual(output.info, original.info)
        self.assertEqual(output.data, original.data)
        self.assertIsNot(output, original)

    def test_input_and_output_topics_must_be_distinct(self):
        validate_topic_separation(("/a", "/b", "/c", "/d"))
        with self.assertRaises(ValueError):
            validate_topic_separation(("/a", "/b", "/a", "/d"))
        with self.assertRaises(ValueError):
            validate_topic_separation(("/a", "", "/c", "/d"))


class EnvelopeMonitorTests(unittest.TestCase):
    def test_decisions_have_readable_names(self):
        self.assertEqual(decision_name(SafetyEnvelope.STOP), "STOP")
        self.assertEqual(decision_name(SafetyEnvelope.SLOW), "SLOW")
        self.assertEqual(decision_name(SafetyEnvelope.CLEAR), "CLEAR")
        self.assertEqual(decision_name(99), "UNKNOWN(99)")

    def test_signature_ignores_heartbeat_fields(self):
        first = SafetyEnvelope()
        first.decision = SafetyEnvelope.SLOW
        first.reason = "obstacle_slow"
        first.permitted_forward = 0.35
        first.map_age_ms = 10.0
        first.intent_sequence = 1
        second = SafetyEnvelope()
        second.decision = first.decision
        second.reason = first.reason
        second.permitted_forward = first.permitted_forward
        second.map_age_ms = 20.0
        second.intent_sequence = 2

        self.assertEqual(envelope_signature(first), envelope_signature(second))
        self.assertIn("SLOW reason=obstacle_slow", format_envelope(first))


if __name__ == "__main__":
    unittest.main()
