import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from perception.velocity_tracker import (  # noqa: E402
    PersonObservation,
    PersonVelocityTracker,
    center_from_keypoints,
)


class VelocityTrackerTests(unittest.TestCase):
    def test_center_from_keypoints_uses_confident_points(self):
        keypoints = np.array(
            [
                [0.0, 0.0],
                [10.0, 10.0],
                [20.0, 20.0],
            ]
        )
        confidence = np.array([0.9, 0.1, 0.9])

        center = center_from_keypoints(keypoints, confidence, min_keypoint_conf=0.3)

        np.testing.assert_allclose(center, np.array([20.0, 20.0]))

    def test_velocity_from_two_frames_with_track_id(self):
        tracker = PersonVelocityTracker(history_size=2)

        first = PersonObservation(
            center_xy=np.array([100.0, 100.0]),
            keypoints_xy=np.empty((0, 2)),
            track_id=7,
        )
        second = PersonObservation(
            center_xy=np.array([120.0, 110.0]),
            keypoints_xy=np.empty((0, 2)),
            track_id=7,
        )

        self.assertEqual(tracker.update([first], timestamp_s=0.0), [])
        estimates = tracker.update([second], timestamp_s=0.5)

        self.assertEqual(len(estimates), 1)
        self.assertEqual(estimates[0].track_id, 7)
        np.testing.assert_allclose(estimates[0].velocity_xy_px_s, np.array([40.0, 20.0]))

    def test_nearest_centroid_fallback_assigns_same_track(self):
        tracker = PersonVelocityTracker(history_size=2, max_match_distance_px=50.0)

        first = PersonObservation(
            center_xy=np.array([50.0, 50.0]),
            keypoints_xy=np.empty((0, 2)),
        )
        second = PersonObservation(
            center_xy=np.array([60.0, 50.0]),
            keypoints_xy=np.empty((0, 2)),
        )

        tracker.update([first], timestamp_s=1.0)
        estimates = tracker.update([second], timestamp_s=2.0)

        self.assertEqual(len(estimates), 1)
        self.assertEqual(estimates[0].track_id, 1)
        np.testing.assert_allclose(estimates[0].velocity_xy_px_s, np.array([10.0, 0.0]))


if __name__ == "__main__":
    unittest.main()
