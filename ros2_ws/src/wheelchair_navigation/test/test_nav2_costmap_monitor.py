import unittest

from wheelchair_navigation.nav2_costmap_monitor import (
    ArrivalStats,
    continuity_summary,
    percentile,
)


class ArrivalStatsTests(unittest.TestCase):
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


if __name__ == "__main__":
    unittest.main()
