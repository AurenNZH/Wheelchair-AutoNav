import unittest

import numpy as np

from wheelchair_shared_control.models import (
    CLEAR,
    SLOW,
    STOP,
    OperatorIntentData,
    SafetyConfig,
    weighted_costmap_from_grid,
)
from wheelchair_shared_control.safety_policy import (
    evaluate_safety,
    motion_configuration_decision,
)
from wheelchair_shared_control.operator_intent import (
    FORWARD,
    FORWARD_LEFT,
    FORWARD_RIGHT,
    LEFT_TURN,
    REVERSE,
    REVERSE_LEFT,
    REVERSE_RIGHT,
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
            stop_distance_m=0.7,
            slow_distance_m=1.2,
            slow_cost_threshold=1,
            stop_cost_threshold=99,
        )
        self.empty = self._costmap({})

    @staticmethod
    def _costmap(cells, *, width=40, height=80, origin_x=0.0):
        values = np.zeros(width * height, dtype=np.int16)
        for (col, row), cost in cells.items():
            values[row * width + col] = cost
        return weighted_costmap_from_grid(
            values,
            frame_id="base_link",
            width=width,
            height=height,
            resolution_m=0.1,
            origin_x_m=origin_x,
            origin_y_m=-4.0,
            origin_orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
        )

    def test_defaults_are_fail_closed(self):
        decision = evaluate_safety(
            self.intent, self.empty, SafetyConfig()
        )
        self.assertEqual(decision.decision, STOP)
        self.assertEqual(decision.reason, "live_control_disabled")

    def test_uncalibrated_geometry_remains_stopped(self):
        config = SafetyConfig(enable_motion=True, geometry_calibrated=False)
        decision = evaluate_safety(self.intent, self.empty, config)
        self.assertEqual(decision.reason, "uncalibrated_geometry")

    def test_motion_configuration_gate_preserves_reason_precedence(self):
        disabled = motion_configuration_decision(SafetyConfig())
        uncalibrated = motion_configuration_decision(
            SafetyConfig(enable_motion=True)
        )
        enabled = motion_configuration_decision(self.enabled)

        self.assertEqual(disabled.reason, "live_control_disabled")
        self.assertEqual(uncalibrated.reason, "uncalibrated_geometry")
        self.assertIsNone(enabled)

    def test_clear_forward_intent_is_preserved(self):
        decision = evaluate_safety(
            self.intent, self.empty, self.enabled
        )
        self.assertEqual(decision.decision, CLEAR)
        self.assertEqual(decision.reason, "nav2_cost_clear")
        self.assertAlmostEqual(decision.permitted_forward, 0.5)
        self.assertEqual(decision.maximum_path_cost, 0)
        self.assertTrue(decision.path_cost_valid)

    def test_forward_policy_accepts_rear_extended_merged_map(self):
        merged = self._costmap({}, width=50, origin_x=-0.6)

        decision = evaluate_safety(self.intent, merged, self.enabled)

        self.assertEqual(decision.decision, CLEAR)
        self.assertEqual(decision.reason, "nav2_cost_clear")
        self.assertTrue(decision.path_cost_valid)

    def test_reverse_cone_is_unmonitored_and_capped_slow(self):
        fixtures = (
            (0.0, -1.0, REVERSE, 0.0),
            (0.2, -1.0, REVERSE_LEFT, 0.2),
            (-0.2, -1.0, REVERSE_RIGHT, -0.2),
        )
        blocked_front = self._costmap({(1, 40): 100})
        for lateral, longitudinal, intent_class, steering in fixtures:
            with self.subTest(intent_class=intent_class):
                decision = evaluate_safety(
                    OperatorIntentData(
                        "session", 2, lateral, longitudinal,
                        intent_class, True
                    ),
                    blocked_front,
                    self.enabled,
                )
                self.assertEqual(decision.decision, SLOW)
                self.assertEqual(decision.reason, "reverse_unmonitored_slow")
                self.assertAlmostEqual(decision.permitted_forward, 0.65)
                self.assertAlmostEqual(decision.permitted_steering, steering)
                self.assertFalse(decision.path_cost_valid)

    def test_symmetric_thirty_degree_corrections_reach_policy(self):
        fixtures = (
            (-0.57, 1.0, FORWARD_RIGHT, CLEAR),
            (0.57, 1.0, FORWARD_LEFT, CLEAR),
            (-0.57, -1.0, REVERSE_RIGHT, SLOW),
            (0.57, -1.0, REVERSE_LEFT, SLOW),
        )
        for lateral, longitudinal, intent_class, expected in fixtures:
            with self.subTest(intent_class=intent_class):
                decision = evaluate_safety(
                    OperatorIntentData(
                        "session", 20, lateral, longitudinal,
                        intent_class, True
                    ),
                    self.empty,
                    self.enabled,
                )
                self.assertEqual(decision.decision, expected)

    def test_cost_bands_produce_stop_slow_and_clear(self):
        fast_intent = OperatorIntentData(
            "session", 8, 0.0, 1.0, FORWARD, True
        )
        stop = evaluate_safety(
            self.intent,
            self._costmap({(6, 40): 99}),
            self.enabled,
        )
        slow = evaluate_safety(
            fast_intent,
            self._costmap({(10, 40): 50}),
            self.enabled,
        )
        clear = evaluate_safety(
            self.intent,
            self._costmap({(15, 40): 100}),
            self.enabled,
        )

        self.assertEqual(stop.decision, STOP)
        self.assertEqual(stop.reason, "nav2_cost_stop")
        self.assertLessEqual(stop.nearest_stop_cost_distance_m, 0.7)
        self.assertEqual(slow.decision, SLOW)
        self.assertEqual(slow.reason, "nav2_cost_slow")
        self.assertEqual(slow.maximum_path_cost, 50)
        self.assertAlmostEqual(slow.permitted_forward, 0.60)
        self.assertEqual(clear.decision, CLEAR)

    def test_cost_98_slows_while_99_stops_at_same_location(self):
        slow = evaluate_safety(
            self.intent,
            self._costmap({(6, 40): 98}),
            self.enabled,
        )
        stop = evaluate_safety(
            self.intent,
            self._costmap({(6, 40): 99}),
            self.enabled,
        )

        self.assertEqual(slow.decision, SLOW)
        self.assertIsNone(slow.nearest_stop_cost_distance_m)
        self.assertEqual(stop.decision, STOP)

    def test_binary_costmap_remains_valid_transport_input(self):
        decision = evaluate_safety(
            self.intent,
            self._costmap({(10, 40): 100}),
            self.enabled,
        )

        self.assertEqual(decision.decision, SLOW)
        self.assertEqual(decision.maximum_path_cost, 100)

    def test_requested_correction_checks_curved_cost_trajectory(self):
        correction = OperatorIntentData(
            "session", 3, -0.175, 0.5, FORWARD_RIGHT, True
        )
        costs = self._costmap({(9, 38): 50})

        turn = evaluate_safety(correction, costs, self.enabled)
        straight = evaluate_safety(self.intent, costs, self.enabled)

        self.assertEqual(turn.decision, SLOW)
        self.assertEqual(straight.decision, CLEAR)

    def test_correction_includes_straight_path_union(self):
        correction = OperatorIntentData(
            "session", 4, 0.175, 0.5, FORWARD_LEFT, True
        )
        decision = evaluate_safety(
            correction,
            self._costmap({(10, 40): 50}),
            self.enabled,
        )

        self.assertEqual(decision.decision, SLOW)

    def test_irrelevant_side_cost_is_ignored(self):
        decision = evaluate_safety(
            self.intent,
            self._costmap({(6, 20): 100}),
            self.enabled,
        )

        self.assertEqual(decision.decision, CLEAR)
        self.assertEqual(decision.maximum_path_cost, 0)

    def test_unknown_and_out_of_bounds_are_fail_closed(self):
        unknown = evaluate_safety(
            self.intent,
            self._costmap({(5, 40): -1}),
            self.enabled,
        )
        short_map = self._costmap({}, width=5)
        outside = evaluate_safety(
            self.intent, short_map, self.enabled
        )

        self.assertEqual(unknown.reason, "unknown_nav2_cost")
        self.assertFalse(unknown.path_cost_valid)
        self.assertEqual(outside.reason, "trajectory_outside_costmap")
        self.assertFalse(outside.path_cost_valid)

    def test_hard_left_and_right_turns_are_vetoed(self):
        left = evaluate_safety(
            OperatorIntentData(
                "session", 5, 0.96, 0.12, LEFT_TURN, True
            ),
            self.empty,
            self.enabled,
        )
        right = evaluate_safety(
            OperatorIntentData(
                "session", 6, -1.0, 0.0, RIGHT_TURN, True
            ),
            self.empty,
            self.enabled,
        )

        self.assertEqual(left.reason, "left_turn_not_enabled")
        self.assertEqual(right.reason, "right_turn_not_enabled")

    def test_intent_class_mismatch_is_stopped(self):
        mismatch = evaluate_safety(
            OperatorIntentData(
                "session", 7, 0.96, 0.12, FORWARD_LEFT, True
            ),
            self.empty,
            self.enabled,
        )
        self.assertEqual(mismatch.reason, "intent_class_mismatch")


if __name__ == "__main__":
    unittest.main()
