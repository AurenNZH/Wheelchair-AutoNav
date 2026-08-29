import json
import unittest

from wheelchair_shared_control.protocol import (
    EnvelopePacket,
    IntentPacket,
    ProtocolError,
    decode_envelope,
    decode_intent,
    encode_envelope,
    encode_intent,
)
from wheelchair_shared_control.operator_intent import FORWARD_LEFT


class ProtocolTests(unittest.TestCase):
    def test_intent_round_trip(self):
        packet = IntentPacket(
            "session-a", 7, 0.2, 0.4, FORWARD_LEFT, True, 0.15
        )
        self.assertEqual(decode_intent(encode_intent(packet)), packet)

    def test_envelope_round_trip(self):
        packet = EnvelopePacket("session-a", 7, 1, 0.25, -0.2, "slow", 142.0)
        self.assertEqual(decode_envelope(encode_envelope(packet)), packet)

    def test_stop_envelope_cannot_allow_motion(self):
        with self.assertRaises(ProtocolError):
            encode_envelope(
                EnvelopePacket("session-a", 7, 0, 0.1, 0.0, "bad", 10.0)
            )

    def test_wrong_version_and_out_of_range_intent_are_rejected(self):
        wrong_version = json.dumps(
            {
                "v": 2,
                "type": "intent",
                "session": "s",
                "seq": 1,
                "lateral": 0.0,
                "longitudinal": 0.0,
                "intent_class": 0,
                "deadman": False,
            }
        ).encode()
        with self.assertRaises(ProtocolError):
            decode_intent(wrong_version)
        with self.assertRaises(ProtocolError):
            encode_intent(
                IntentPacket("s", 1, 0.0, 1.1, FORWARD_LEFT, True)
            )

    def test_boolean_is_not_accepted_as_numeric_sequence(self):
        data = json.dumps(
            {
                "v": 3,
                "type": "intent",
                "session": "s",
                "seq": True,
                "lateral": 0.0,
                "longitudinal": 0.0,
                "max_steering_assist": 0.0,
                "intent_class": 0,
                "deadman": False,
            }
        ).encode()
        with self.assertRaises(ProtocolError):
            decode_intent(data)

    def test_deadman_and_class_must_agree(self):
        with self.assertRaises(ProtocolError):
            encode_intent(IntentPacket("s", 1, 0.0, 0.0, 0, True))
        with self.assertRaises(ProtocolError):
            encode_intent(
                IntentPacket("s", 1, 0.0, 0.5, FORWARD_LEFT, False)
            )

    def test_assist_authority_must_be_explicit_and_normalized(self):
        with self.assertRaises(ProtocolError):
            encode_intent(
                IntentPacket("s", 1, 0.0, 0.5, FORWARD_LEFT, True, 1.01)
            )
        payload = json.loads(
            encode_intent(
                IntentPacket("s", 1, 0.0, 0.5, FORWARD_LEFT, True)
            )
        )
        self.assertEqual(payload["v"], 3)
        self.assertEqual(payload["max_steering_assist"], 0.0)


if __name__ == "__main__":
    unittest.main()
