import unittest

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

from wheelchair_navigation.mapping_latency_recorder import (
    SUPERVISOR_STATUS_NAME,
    SupervisorStaleTracker,
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
        target_failure = latency_summary([251.0] * 100, [100.0] * 99)
        deadline_failure = latency_summary(
            [100.0] * 100 + [301.0], [100.0] * 100
        )

        self.assertTrue(passing["passes"])
        self.assertEqual(passing["over_deadline_count"], 0)
        self.assertFalse(target_failure["passes"])
        self.assertEqual(target_failure["over_deadline_count"], 0)
        self.assertFalse(deadline_failure["passes"])
        self.assertEqual(deadline_failure["over_deadline_count"], 1)

    def test_summary_rejects_arrival_gap_despite_fresh_maps(self):
        summary = latency_summary(
            [150.0, 160.0, 170.0], [100.0, 301.0]
        )

        self.assertFalse(summary["passes"])
        self.assertEqual(summary["arrival_over_deadline_count"], 1)

    def test_summary_rejects_uniformly_slow_map_rate(self):
        summary = latency_summary(
            [150.0, 160.0, 170.0], [125.0, 125.0]
        )

        self.assertFalse(summary["passes"])
        self.assertAlmostEqual(summary["map_rate_hz"], 8.0)

    def test_summary_rejects_stale_counters_and_invalid_diagnostics(self):
        stale_receipt = latency_summary(
            [150.0], [], stale_map_receipt_delta=1
        )
        stale_event = latency_summary(
            [150.0], [], stale_map_event_delta=1
        )
        missing_diagnostics = latency_summary(
            [150.0], [], supervisor_diagnostics_valid=False
        )

        self.assertFalse(stale_receipt["passes"])
        self.assertFalse(stale_event["passes"])
        self.assertFalse(missing_diagnostics["passes"])

    def test_stale_tracker_baselines_warmup_events(self):
        tracker = SupervisorStaleTracker()
        tracker.observe_warmup(
            {
                "stale_map_receipt_count": "4",
                "stale_map_event_count": "7",
            }
        )
        tracker.begin_capture()
        self.assertEqual(tracker.result(), (0, 0, False))

        tracker.observe_capture(
            {
                "stale_map_receipt_count": "4",
                "stale_map_event_count": "7",
            }
        )

        self.assertEqual(tracker.result(), (0, 0, True))

    def test_stale_tracker_reports_capture_deltas(self):
        tracker = SupervisorStaleTracker()
        tracker.observe_warmup(
            {
                "stale_map_receipt_count": "2",
                "stale_map_event_count": "3",
            }
        )
        tracker.observe_capture(
            {
                "stale_map_receipt_count": "5",
                "stale_map_event_count": "4",
            }
        )

        self.assertEqual(tracker.result(), (3, 1, True))

    def test_stale_tracker_rejects_missing_malformed_and_reset_counters(self):
        missing_baseline = SupervisorStaleTracker()
        missing_baseline.begin_capture()
        missing_baseline.observe_capture(
            {
                "stale_map_receipt_count": "0",
                "stale_map_event_count": "0",
            }
        )
        self.assertEqual(missing_baseline.result(), (0, 0, False))

        malformed = SupervisorStaleTracker()
        malformed.observe_warmup(
            {
                "stale_map_receipt_count": "2",
                "stale_map_event_count": "3",
            }
        )
        malformed.observe_capture(
            {
                "stale_map_receipt_count": "not-an-integer",
                "stale_map_event_count": "3",
            }
        )
        self.assertEqual(malformed.result(), (0, 0, False))

        reset = SupervisorStaleTracker()
        reset.observe_warmup(
            {
                "stale_map_receipt_count": "5",
                "stale_map_event_count": "7",
            }
        )
        reset.observe_capture(
            {
                "stale_map_receipt_count": "0",
                "stale_map_event_count": "0",
            }
        )
        self.assertEqual(reset.result(), (0, 0, False))

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
