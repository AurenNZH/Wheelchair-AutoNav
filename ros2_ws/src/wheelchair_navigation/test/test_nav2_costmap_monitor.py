import unittest

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from wheelchair_navigation.nav2_costmap_monitor import (
    ArrivalStats,
    artifact_filter_diagnostic_values,
    continuity_summary,
    filtered_continuity_summary,
    percentile,
)


class ArrivalStatsTests(unittest.TestCase):
    def test_extracts_only_artifact_filter_diagnostics(self):
        message = DiagnosticArray()
        unrelated = DiagnosticStatus()
        unrelated.name = "other"
        filtered = DiagnosticStatus()
        filtered.name = "wheelchair_navigation/artifact_point_filter"
        filtered.values = [
            KeyValue(key="received_clouds", value="12"),
            KeyValue(key="published_clouds", value="12"),
        ]
        message.status = [unrelated, filtered]

        self.assertEqual(
            artifact_filter_diagnostic_values(message),
            {"received_clouds": "12", "published_clouds": "12"},
        )

    def test_rate_and_maximum_gap(self):
        stats = ArrivalStats()
        for arrival in (10.0, 10.1, 10.2, 10.4):
            stats.observe(arrival)

        self.assertEqual(stats.count, 4)
        self.assertAlmostEqual(stats.rate_hz, 7.5)
        self.assertAlmostEqual(stats.maximum_gap_ms, 200.0)

    def test_percentile_uses_linear_interpolation(self):
        self.assertEqual(percentile([], 99.0), None)
        self.assertAlmostEqual(percentile([0.0, 100.0], 50.0), 50.0)

    def test_continuity_pass_requires_both_streams(self):
        cloud = ArrivalStats()
        costmap = ArrivalStats()
        for index in range(40):
            cloud.observe(index * 0.1)
            costmap.observe(index * 0.1 + 0.02)

        result = continuity_summary(
            cloud,
            costmap,
            minimum_map_rate_hz=9.0,
            maximum_gap_ms=300.0,
        )

        self.assertTrue(result["passes"])
        self.assertAlmostEqual(result["map_rate_hz"], 10.0)

    def test_continuity_fails_on_map_freeze(self):
        cloud = ArrivalStats()
        costmap = ArrivalStats()
        for index in range(40):
            cloud.observe(index * 0.1)
            costmap.observe(index * 0.1)
        costmap.observe(4.5)

        result = continuity_summary(
            cloud,
            costmap,
            minimum_map_rate_hz=9.0,
            maximum_gap_ms=300.0,
        )

        self.assertFalse(result["passes"])
        self.assertAlmostEqual(result["map_gap_max_ms"], 600.0)

    def test_filtered_summary_requires_rate_ratio_and_map_continuity(self):
        raw = ArrivalStats()
        filtered = ArrivalStats()
        costmap = ArrivalStats()
        for index in range(40):
            raw.observe(index * 0.1)
            filtered.observe(index * 0.1 + 0.01)
            costmap.observe(index * 0.15)

        result = filtered_continuity_summary(
            raw,
            filtered,
            costmap,
            minimum_map_rate_hz=5.7,
            maximum_gap_ms=300.0,
            minimum_filtered_ratio=0.9,
        )

        self.assertTrue(result["passes"])
        self.assertAlmostEqual(result["filtered_rate_ratio"], 1.0)

    def test_filtered_summary_fails_when_filter_drops_stream(self):
        raw = ArrivalStats()
        filtered = ArrivalStats()
        costmap = ArrivalStats()
        for index in range(40):
            raw.observe(index * 0.1)
            costmap.observe(index * 0.15)
            if index % 2 == 0:
                filtered.observe(index * 0.1 + 0.01)

        result = filtered_continuity_summary(
            raw,
            filtered,
            costmap,
            minimum_map_rate_hz=5.7,
            maximum_gap_ms=300.0,
            minimum_filtered_ratio=0.9,
        )

        self.assertFalse(result["passes"])
        self.assertLess(result["filtered_rate_ratio"], 0.9)


if __name__ == "__main__":
    unittest.main()
