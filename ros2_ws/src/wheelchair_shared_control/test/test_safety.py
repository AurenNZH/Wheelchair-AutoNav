import unittest

import numpy as np

from wheelchair_shared_control.safety import (
    CLEAR,
    SLOW,
    STOP,
    OperatorIntentData,
    SafetyConfig,
    evaluate_safety,
    occupied_points_from_grid,
)


class SafetyPolicyTests(unittest.TestCase):
    def setUp(self):
        self.intent = OperatorIntentData("session", 1, 0.0, 0.5, True)
        self.enabled = SafetyConfig(
            enable_motion=True,
            geometry_calibrated=True,
            chair_width_m=0.7,
            front_extent_m=0.8,
            rear_extent_m=0.4,
            lateral_margin_m=0.15,
            stop_distance_m=0.7,
            slow_distance_m=1.2,
        )
        self.empty = np.empty((0, 2), dtype=np.float32)

    def test_defaults_are_fail_closed(self):
        decision = evaluate_safety(
            self.intent, self.empty, self.empty, 0.1, SafetyConfig()
        )
        self.assertEqual(decision.decision, STOP)
        self.assertEqual(decision.reason, "live_control_disabled")

    def test_uncalibrated_geometry_remains_stopped(self):
        config = SafetyConfig(enable_motion=True, geometry_calibrated=False)
        decision = evaluate_safety(
            self.intent, self.empty, self.empty, 0.1, config
        )
        self.assertEqual(decision.reason, "uncalibrated_geometry")

    def test_clear_forward_intent_is_preserved(self):
        decision = evaluate_safety(
            self.intent, self.empty, self.empty, 0.1, self.enabled
        )
        self.assertEqual(decision.decision, CLEAR)
        self.assertAlmostEqual(decision.permitted_forward, 0.5)

    def test_stale_map_and_reverse_are_stopped(self):
        stale = evaluate_safety(
            self.intent, self.empty, self.empty, 0.31, self.enabled
        )
        reverse = evaluate_safety(
            OperatorIntentData("session", 2, 0.0, -0.1, True),
            self.empty,
            self.empty,
            0.1,
            self.enabled,
        )
        self.assertEqual(stale.reason, "stale_map")
        self.assertEqual(reverse.reason, "reverse_disabled")

    def test_obstacle_in_stop_and_slow_regions(self):
        stop = evaluate_safety(
            self.intent,
            np.array([[1.0, 0.0]], dtype=np.float32),
            self.empty,
            0.1,
            self.enabled,
        )
        slow = evaluate_safety(
            self.intent,
            np.array([[1.7, 0.0]], dtype=np.float32),
            self.empty,
            0.1,
            self.enabled,
        )
        self.assertEqual(stop.decision, STOP)
        self.assertEqual(stop.reason, "obstacle_stop")
        self.assertEqual(slow.decision, SLOW)
        self.assertLess(slow.permitted_forward, self.intent.forward)

    def test_requested_turn_checks_curved_swept_footprint(self):
        turning_intent = OperatorIntentData("session", 3, -0.35, 0.5, True)
        obstacle = np.array([[1.3, -0.65]], dtype=np.float32)
        turn = evaluate_safety(
            turning_intent, obstacle, self.empty, 0.1, self.enabled
        )
        straight = evaluate_safety(
            self.intent, obstacle, self.empty, 0.1, self.enabled
        )
        self.assertIn(turn.decision, (STOP, SLOW))
        self.assertEqual(straight.decision, CLEAR)

    def test_left_and_excessive_right_turns_are_vetoed(self):
        left = evaluate_safety(
            OperatorIntentData("session", 4, 0.1, 0.5, True),
            self.empty,
            self.empty,
            0.1,
            self.enabled,
        )
        excessive_right = evaluate_safety(
            OperatorIntentData("session", 5, -0.5, 0.5, True),
            self.empty,
            self.empty,
            0.1,
            self.enabled,
        )

        self.assertEqual(left.decision, STOP)
        self.assertEqual(left.reason, "left_turn_unobserved")
        self.assertEqual(excessive_right.decision, STOP)
        self.assertEqual(excessive_right.reason, "right_turn_limit_exceeded")

    def test_full_surround_proximity_vetoes_motion(self):
        decision = evaluate_safety(
            self.intent,
            self.empty,
            np.array([[-0.2, 0.45]], dtype=np.float32),
            0.1,
            self.enabled,
        )
        self.assertEqual(decision.reason, "surround_proximity")

    def test_occupancy_grid_conversion_uses_cell_centres(self):
        points = occupied_points_from_grid(
            [0, 100, 0, 0],
            width=2,
            height=2,
            resolution_m=0.1,
            origin_x_m=0.0,
            origin_y_m=-0.1,
        )
        np.testing.assert_allclose(points, [[0.15, -0.05]])


if __name__ == "__main__":
    unittest.main()
