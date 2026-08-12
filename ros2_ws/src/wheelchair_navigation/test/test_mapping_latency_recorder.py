import unittest

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

from wheelchair_navigation.mapping_latency_recorder import (
    SUPERVISOR_STATUS_NAME,
    find_status,
    latency_summary,
    percentile,
)


class MappingLatencyRecorderTests(unittest.TestCase):
    def test_percentile_ignores_nonfinite_values(self):
        result = percentile([10.0, float("nan"), 30.0], 50)
        self.assertAlmostEqual(result, 20.0)
        self.assertIsNone(percentile([], 99))

    def test_summary_requires_p99_margin_and_zero_deadline_misses(self):
        passing = latency_summary([100.0, 200.0, 240.0], [100.0, 100.0])
        deadline_failure = latency_summary(
            [100.0] * 100 + [301.0], [100.0] * 100
        )

        self.assertTrue(passing["passes"])
        self.assertEqual(passing["over_deadline_count"], 0)
        self.assertFalse(deadline_failure["passes"])
        self.assertEqual(deadline_failure["over_deadline_count"], 1)

    def test_finds_named_supervisor_status(self):
        expected = DiagnosticStatus()
        expected.name = SUPERVISOR_STATUS_NAME
        other = DiagnosticStatus()
        other.name = "other"
        msg = DiagnosticArray()
        msg.status = [other, expected]

        self.assertIs(find_status(msg, SUPERVISOR_STATUS_NAME), expected)


if __name__ == "__main__":
    unittest.main()
