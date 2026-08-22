import unittest

from wheelchair_navigation.timing import (
    CloudTimingTracker,
    cloud_age_ms,
    cloud_timestamp_error,
)


class TimingTests(unittest.TestCase):
    def test_cloud_timestamp_accepts_recent_data(self):
        self.assertIsNone(
            cloud_timestamp_error(
                now_ns=2_000_000_000,
                stamp_ns=1_500_000_000,
                max_age_s=1.0,
                max_future_offset_s=0.1,
            )
        )

    def test_cloud_timestamp_rejects_stale_future_and_invalid_data(self):
        common = {
            "now_ns": 2_000_000_000,
            "max_age_s": 1.0,
            "max_future_offset_s": 0.1,
        }
        self.assertEqual(
            cloud_timestamp_error(stamp_ns=0, **common),
            "invalid_lidar_timestamp",
        )
        self.assertEqual(
            cloud_timestamp_error(stamp_ns=500_000_000, **common),
            "stale_lidar",
        )
        self.assertEqual(
            cloud_timestamp_error(stamp_ns=2_200_000_000, **common),
            "future_lidar",
        )

    def test_tracker_preserves_first_and_non_monotonic_period_behavior(self):
        tracker = CloudTimingTracker()

        first = tracker.record(1_000_000_000, 5.0)
        second = tracker.record(1_100_000_000, 5.25)
        reversed_sample = tracker.record(1_050_000_000, 5.20)

        self.assertEqual((first.source_ms, first.arrival_ms), (0.0, 0.0))
        self.assertAlmostEqual(second.source_ms, 100.0)
        self.assertAlmostEqual(second.arrival_ms, 250.0)
        self.assertEqual(
            (reversed_sample.source_ms, reversed_sample.arrival_ms),
            (0.0, 0.0),
        )

    def test_cloud_age_is_non_negative_and_invalid_stamps_are_zero(self):
        self.assertEqual(cloud_age_ms(2_000_000_000, 0), 0.0)
        self.assertEqual(cloud_age_ms(2_000_000_000, 2_100_000_000), 0.0)
        self.assertEqual(cloud_age_ms(2_000_000_000, 1_750_000_000), 250.0)


if __name__ == "__main__":
    unittest.main()
