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
from wheelchair_shared_control.operator_intent import (
    FORWARD,
    FORWARD_LEFT,
    FORWARD_RIGHT,
    LEFT_TURN,
    REVERSE,
    RIGHT_TURN,
)


class SafetyPolicyTests(unittest.TestCase):
    def setUp(self):
        self.intent = OperatorIntentData(
            "session", 1, 0.0, 0.5, FORWARD, True
        )
        self.enabled = SafetyConfig(
            enable_motion=True,
            geometry_calibrated=True,
            chair_width_m=0.7,
            front_extent_m=0.4,
            rear_extent_m=0.4,
            lateral_margin_m=0.15,
            stop_distance_m=0.7,
            slow_distance_m=1.2,
        )
        self.empty = np.empty((0, 2), dtype=np.float32)

    def test_defaults_are_fail_closed(self):
        decision = evaluate_safety(
            self.intent, self.empty, 0.1, SafetyConfig()
        )
        self.assertEqual(decision.decision, STOP)
        self.assertEqual(decision.reason, "live_control_disabled")

    def test_uncalibrated_geometry_remains_stopped(self):
        config = SafetyConfig(enable_motion=True, geometry_calibrated=False)
        decision = evaluate_safety(
            self.intent, self.empty, 0.1, config
        )
        self.assertEqual(decision.reason, "uncalibrated_geometry")

    def test_clear_forward_intent_is_preserved(self):
        decision = evaluate_safety(
            self.intent, self.empty, 0.1, self.enabled
        )
        self.assertEqual(decision.decision, CLEAR)
        self.assertAlmostEqual(decision.permitted_forward, 0.5)

    def test_stale_map_and_reverse_are_stopped(self):
        stale = evaluate_safety(
            self.intent, self.empty, 0.31, self.enabled
        )
        reverse = evaluate_safety(
            OperatorIntentData(
                "session", 2, 0.0, -0.1, REVERSE, True
            ),
            self.empty,
            0.1,
            self.enabled,
        )
        self.assertEqual(stale.reason, "stale_map")
        self.assertEqual(reverse.reason, "reverse_not_enabled")

    def test_obstacle_in_stop_and_slow_regions(self):
        stop = evaluate_safety(
            self.intent,
            np.array([[1.0, 0.0]], dtype=np.float32),
            0.1,
            self.enabled,
        )
        slow = evaluate_safety(
            self.intent,
            np.array([[1.4, 0.0]], dtype=np.float32),
            0.1,
            self.enabled,
        )
        self.assertEqual(stop.decision, STOP)
        self.assertEqual(stop.reason, "obstacle_stop")
        self.assertEqual(slow.decision, SLOW)
        self.assertLess(slow.permitted_forward, self.intent.longitudinal)

    def test_centred_base_uses_four_tenths_forward_extent(self):
        current_footprint = evaluate_safety(
            self.intent,
            np.array([[0.4, 0.0]], dtype=np.float32),
            0.1,
            self.enabled,
        )
        ahead = evaluate_safety(
            self.intent,
            np.array([[1.2, 0.0]], dtype=np.float32),
            0.1,
            self.enabled,
        )

        self.assertEqual(current_footprint.nearest_path_distance_m, 0.0)
        self.assertAlmostEqual(
            ahead.nearest_path_distance_m,
            0.8,
            delta=self.enabled.path_sample_step_m + 1e-6,
        )
        self.assertEqual(ahead.decision, SLOW)

    def test_requested_turn_checks_curved_swept_footprint(self):
        turning_intent = OperatorIntentData(
            "session", 3, -0.175, 0.5, FORWARD_RIGHT, True
        )
        obstacle = np.array([[1.3, -0.65]], dtype=np.float32)
        turn = evaluate_safety(
            turning_intent, obstacle, 0.1, self.enabled
        )
        straight = evaluate_safety(
            self.intent, obstacle, 0.1, self.enabled
        )
        self.assertIn(turn.decision, (STOP, SLOW))
        self.assertEqual(straight.decision, CLEAR)

    def test_hard_left_and_right_turns_are_vetoed(self):
        left = evaluate_safety(
            OperatorIntentData(
                "session", 4, 0.96, 0.12, LEFT_TURN, True
            ),
            self.empty,
            0.1,
            self.enabled,
        )
        right = evaluate_safety(
            OperatorIntentData(
                "session", 5, -1.0, 0.0, RIGHT_TURN, True
            ),
            self.empty,
            0.1,
            self.enabled,
        )

        self.assertEqual(left.decision, STOP)
        self.assertEqual(left.reason, "left_turn_not_enabled")
        self.assertEqual(right.decision, STOP)
        self.assertEqual(right.reason, "right_turn_not_enabled")

    def test_intent_class_mismatch_is_stopped(self):
        mismatch = evaluate_safety(
            OperatorIntentData(
                "session", 5, 0.96, 0.12, FORWARD_LEFT, True
            ),
            self.empty,
            0.1,
            self.enabled,
        )
        self.assertEqual(mismatch.reason, "intent_class_mismatch")

    def test_correction_checks_straight_to_requested_path_union(self):
        correction = OperatorIntentData(
            "session", 6, 0.175, 0.5, FORWARD_LEFT, True
        )
        obstacle_on_straight = np.array([[1.3, -0.45]], dtype=np.float32)

        decision = evaluate_safety(
            correction,
            obstacle_on_straight,
            0.1,
            self.enabled,
        )

        self.assertIn(decision.decision, (STOP, SLOW))
        self.assertIn(decision.reason, ("obstacle_stop", "obstacle_slow"))

    def test_front_footprint_obstacle_still_vetoes_motion(self):
        decision = evaluate_safety(
            self.intent,
            np.array([[0.2, 0.45]], dtype=np.float32),
            0.1,
            self.enabled,
        )
        self.assertEqual(decision.reason, "obstacle_stop")
        self.assertEqual(decision.nearest_path_distance_m, 0.0)

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
