import unittest

from wheelchair_navigation.mapping_diagnostics import MappingMetrics


class MappingDiagnosticsTests(unittest.TestCase):
    def test_metrics_report_window_rate_and_latency(self):
        metrics = MappingMetrics(3)
        metrics.record(10.0, 1.0)
        metrics.record(20.0, 1.1)
        metrics.record(30.0, 1.2)

        values = metrics.values(lag_spike_ms=25.0)

        self.assertEqual(values["latency_window_count"], "3")
        self.assertEqual(values["processing_p50_ms"], "20.000")
        self.assertEqual(values["processing_p95_ms"], "29.000")
        self.assertEqual(values["processing_max_ms"], "30.000")
        self.assertEqual(values["mapping_p95_ms"], "29.000")
        self.assertEqual(values["cloud_age_p95_ms"], "0.000")
        self.assertEqual(values["lag_spike_count"], "1")
        self.assertEqual(values["effective_rate_hz"], "10.000")

    def test_window_size_must_be_positive(self):
        with self.assertRaises(ValueError):
            MappingMetrics(0)


if __name__ == "__main__":
    unittest.main()
