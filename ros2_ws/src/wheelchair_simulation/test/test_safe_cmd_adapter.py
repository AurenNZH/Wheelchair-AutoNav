import unittest

from wheelchair_simulation.safe_cmd_adapter import (
    SimEnvelope,
    SimIntent,
    safe_sim_velocity,
)


class SafeSimulationCommandTests(unittest.TestCase):
    def setUp(self):
        self.intent = SimIntent("session", 3, 0.2, 0.5, True)
        self.envelope = SimEnvelope("session", 3, 2, 0.4, 0.1)
        self.common = {
            "intent_age_s": 0.05,
            "envelope_age_s": 0.05,
            "max_age_s": 0.2,
            "max_linear_mps": 0.15,
            "max_angular_radps": 0.4,
        }

    def test_matching_clear_envelope_produces_bounded_simulation_velocity(self):
        linear, angular, reason = safe_sim_velocity(
            self.intent, self.envelope, **self.common
        )
        self.assertAlmostEqual(linear, 0.06)
        self.assertAlmostEqual(angular, 0.04)
        self.assertEqual(reason, "permitted")

    def test_missing_stale_mismatched_and_stop_inputs_produce_zero(self):
        cases = [
            (None, self.envelope, self.common),
            (
                self.intent,
                self.envelope,
                dict(self.common, envelope_age_s=0.21),
            ),
            (
                self.intent,
                SimEnvelope("session", 2, 2, 0.4, 0.1),
                self.common,
            ),
            (
                self.intent,
                SimEnvelope("session", 3, 0, 0.0, 0.0),
                self.common,
            ),
        ]
        for intent, envelope, parameters in cases:
            linear, angular, _ = safe_sim_velocity(
                intent, envelope, **parameters
            )
            self.assertEqual((linear, angular), (0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
