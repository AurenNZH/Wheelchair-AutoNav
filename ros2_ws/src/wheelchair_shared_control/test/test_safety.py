import math
import unittest

import numpy as np

from wheelchair_shared_control.safety import (
    CLEAR,
    SLOW,
    STOP,
    OperatorIntentData,
    SafetyConfig,
    evaluate_right_turn_safety,
    evaluate_safety,
    trajectory_points,
    validate_cost_policy,
    weighted_costmap_from_grid,
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
            self.intent, self.empty, 0.1, SafetyConfig()
        )
        self.assertEqual(decision.decision, STOP)
        self.assertEqual(decision.reason, "live_control_disabled")

    def test_uncalibrated_geometry_remains_stopped(self):
        config = SafetyConfig(enable_motion=True, geometry_calibrated=False)
        decision = evaluate_safety(self.intent, self.empty, 0.1, config)
        self.assertEqual(decision.reason, "uncalibrated_geometry")

    def test_clear_forward_intent_is_preserved(self):
        decision = evaluate_safety(
            self.intent, self.empty, 0.1, self.enabled
        )
        self.assertEqual(decision.decision, CLEAR)
        self.assertEqual(decision.reason, "nav2_cost_clear")
        self.assertAlmostEqual(decision.permitted_forward, 0.5)
        self.assertEqual(decision.maximum_path_cost, 0)
        self.assertTrue(decision.path_cost_valid)

    def test_stale_map_is_stopped(self):
        stale = evaluate_safety(
            self.intent, self.empty, 0.51, self.enabled
        )
        stale_reverse = evaluate_safety(
            OperatorIntentData(
                "session", 2, 0.0, -1.0, REVERSE, True
            ),
            self.empty,
            0.51,
            self.enabled,
        )
        self.assertEqual(stale.reason, "stale_map")
        self.assertEqual(stale_reverse.reason, "stale_map")

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
                    0.1,
                    self.enabled,
                )
                self.assertEqual(decision.decision, SLOW)
                self.assertEqual(decision.reason, "reverse_unmonitored_slow")
                self.assertAlmostEqual(decision.permitted_forward, 0.65)
                self.assertAlmostEqual(decision.permitted_steering, steering)
                self.assertFalse(decision.path_cost_valid)

    def test_cost_bands_produce_stop_slow_and_clear(self):
        fast_intent = OperatorIntentData(
            "session", 8, 0.0, 1.0, FORWARD, True
        )
        stop = evaluate_safety(
            self.intent,
            self._costmap({(6, 40): 99}),
            0.1,
            self.enabled,
        )
        slow = evaluate_safety(
            fast_intent,
            self._costmap({(10, 40): 50}),
            0.1,
            self.enabled,
        )
        clear = evaluate_safety(
            self.intent,
            self._costmap({(15, 40): 100}),
            0.1,
            self.enabled,
        )

        self.assertEqual(stop.decision, STOP)
        self.assertEqual(stop.reason, "nav2_cost_stop")
        self.assertLessEqual(stop.nearest_stop_cost_distance_m, 0.7)
        self.assertEqual(slow.decision, SLOW)
        self.assertEqual(slow.reason, "nav2_cost_slow")
        self.assertEqual(slow.maximum_path_cost, 50)
        self.assertAlmostEqual(slow.permitted_forward, 0.30)
        self.assertEqual(clear.decision, CLEAR)

    def test_cost_98_slows_while_99_stops_at_same_location(self):
        slow = evaluate_safety(
            self.intent,
            self._costmap({(6, 40): 98}),
            0.1,
            self.enabled,
        )
        stop = evaluate_safety(
            self.intent,
            self._costmap({(6, 40): 99}),
            0.1,
            self.enabled,
        )

        self.assertEqual(slow.decision, SLOW)
        self.assertIsNone(slow.nearest_stop_cost_distance_m)
        self.assertEqual(stop.decision, STOP)

    def test_binary_costmap_remains_valid_transport_input(self):
        decision = evaluate_safety(
            self.intent,
            self._costmap({(10, 40): 100}),
            0.1,
            self.enabled,
        )

        self.assertEqual(decision.decision, SLOW)
        self.assertEqual(decision.maximum_path_cost, 100)

    def test_requested_correction_checks_curved_cost_trajectory(self):
        correction = OperatorIntentData(
            "session", 3, -0.175, 0.5, FORWARD_RIGHT, True
        )
        costs = self._costmap({(9, 38): 50})

        turn = evaluate_safety(correction, costs, 0.1, self.enabled)
        straight = evaluate_safety(self.intent, costs, 0.1, self.enabled)

        self.assertEqual(turn.decision, SLOW)
        self.assertEqual(straight.decision, CLEAR)

    def test_forward_right_is_supported_through_thirty_degrees_only(self):
        supported = evaluate_safety(
            OperatorIntentData(
                "session", 30, -0.55, 1.0, FORWARD_RIGHT, True
            ),
            self.empty,
            0.1,
            self.enabled,
        )
        outside = evaluate_safety(
            OperatorIntentData(
                "session", 31, -0.58, 1.0, RIGHT_TURN, True
            ),
            self.empty,
            0.1,
            self.enabled,
        )

        self.assertEqual(supported.decision, CLEAR)
        self.assertEqual(supported.reason, "nav2_cost_clear")
        self.assertEqual(outside.decision, STOP)
        self.assertEqual(outside.reason, "right_turn_not_enabled")

    def test_new_forward_right_sector_checks_its_curved_cells(self):
        steering = -0.55
        point = trajectory_points(steering, self.enabled)[20]
        col = math.floor(point[0] / 0.1)
        row = math.floor((point[1] + 4.0) / 0.1)
        decision = evaluate_safety(
            OperatorIntentData(
                "session", 32, steering, 1.0, FORWARD_RIGHT, True
            ),
            self._costmap({(col, row): 50}),
            0.1,
            self.enabled,
        )

        self.assertEqual(decision.decision, SLOW)
        self.assertEqual(decision.reason, "nav2_cost_slow")
        self.assertIn((col, row), decision.checked_cells)

    def test_correction_includes_straight_path_union(self):
        correction = OperatorIntentData(
            "session", 4, 0.175, 0.5, FORWARD_LEFT, True
        )
        decision = evaluate_safety(
            correction,
            self._costmap({(10, 40): 50}),
            0.1,
            self.enabled,
        )

        self.assertEqual(decision.decision, SLOW)

    def test_irrelevant_side_cost_is_ignored(self):
        decision = evaluate_safety(
            self.intent,
            self._costmap({(6, 20): 100}),
            0.1,
            self.enabled,
        )

        self.assertEqual(decision.decision, CLEAR)
        self.assertEqual(decision.maximum_path_cost, 0)

    def test_unknown_and_out_of_bounds_are_fail_closed(self):
        unknown = evaluate_safety(
            self.intent,
            self._costmap({(5, 40): -1}),
            0.1,
            self.enabled,
        )
        short_map = self._costmap({}, width=5)
        outside = evaluate_safety(
            self.intent, short_map, 0.1, self.enabled
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
            0.1,
            self.enabled,
        )
        right = evaluate_safety(
            OperatorIntentData(
                "session", 6, -1.0, 0.0, RIGHT_TURN, True
            ),
            self.empty,
            0.1,
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
            0.1,
            self.enabled,
        )
        self.assertEqual(mismatch.reason, "intent_class_mismatch")

    def test_invalid_cost_policy_is_rejected(self):
        invalid = (
            SafetyConfig(slow_cost_threshold=0),
            SafetyConfig(slow_cost_threshold=99, stop_cost_threshold=99),
            SafetyConfig(slow_cost_threshold=50, stop_cost_threshold=101),
            SafetyConfig(stop_distance_m=1.3, slow_distance_m=1.2),
            SafetyConfig(path_sample_step_m=math.nan),
            SafetyConfig(slow_forward_limit=0.0),
            SafetyConfig(slow_forward_limit=1.01),
            SafetyConfig(reverse_limit=0.0),
            SafetyConfig(reverse_limit=1.01),
        )
        for config in invalid:
            with self.subTest(config=config):
                with self.assertRaises(ValueError):
                    validate_cost_policy(config)


class RightTurnSafetyPolicyTests(unittest.TestCase):
    @staticmethod
    def _costmap(cells):
        values = np.zeros(80 * 80, dtype=np.int16)
        for (col, row), cost in cells.items():
            values[row * 80 + col] = cost
        return weighted_costmap_from_grid(
            values,
            frame_id="base_link",
            width=80,
            height=80,
            resolution_m=0.1,
            origin_x_m=-4.0,
            origin_y_m=-4.0,
            origin_orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
        )

    def setUp(self):
        self.intent = OperatorIntentData(
            "session", 40, -1.0, 0.0, RIGHT_TURN, True
        )
        self.empty = self._costmap({})
        self.enabled = SafetyConfig(
            enable_motion=True,
            geometry_calibrated=True,
            enable_hard_right_turn=True,
            partial_turn_coverage_acknowledged=True,
            slow_cost_threshold=1,
            stop_cost_threshold=99,
            slow_forward_limit=0.30,
        )

    def test_right_turn_requires_both_explicit_gates(self):
        disabled = evaluate_right_turn_safety(
            self.intent,
            self.empty,
            0.1,
            SafetyConfig(enable_motion=True, geometry_calibrated=True),
        )
        unacknowledged = evaluate_right_turn_safety(
            self.intent,
            self.empty,
            0.1,
            SafetyConfig(
                enable_motion=True,
                geometry_calibrated=True,
                enable_hard_right_turn=True,
            ),
        )

        self.assertEqual(disabled.reason, "right_turn_not_enabled")
        self.assertEqual(
            unacknowledged.reason,
            "partial_turn_coverage_not_acknowledged",
        )

    def test_clear_pivot_preserves_signed_axes(self):
        decision = evaluate_right_turn_safety(
            self.intent, self.empty, 0.1, self.enabled
        )

        self.assertEqual(decision.decision, CLEAR)
        self.assertEqual(decision.reason, "nav2_right_turn_clear")
        self.assertAlmostEqual(decision.permitted_lateral, -1.0)
        self.assertAlmostEqual(decision.permitted_longitudinal, 0.0)
        self.assertTrue(decision.checked_cells)

    def test_pivot_disk_cost_produces_slow_and_stop(self):
        slow = evaluate_right_turn_safety(
            self.intent, self._costmap({(40, 40): 50}), 0.1, self.enabled
        )
        stop = evaluate_right_turn_safety(
            self.intent, self._costmap({(40, 40): 99}), 0.1, self.enabled
        )

        self.assertEqual(slow.decision, SLOW)
        self.assertAlmostEqual(slow.permitted_lateral, -0.30)
        self.assertAlmostEqual(slow.permitted_longitudinal, 0.0)
        self.assertEqual(stop.decision, STOP)
        self.assertEqual(stop.reason, "nav2_right_turn_stop")

    def test_stale_or_reverse_right_request_is_stopped(self):
        stale = evaluate_right_turn_safety(
            self.intent, self.empty, 0.51, self.enabled
        )
        reverse_right = evaluate_right_turn_safety(
            OperatorIntentData(
                "session", 41, -1.0, -0.1, RIGHT_TURN, True
            ),
            self.empty,
            0.1,
            self.enabled,
        )

        self.assertEqual(stale.reason, "stale_turn_map")
        self.assertEqual(reverse_right.reason, "reverse_right_not_enabled")


class WeightedCostmapValidationTests(unittest.TestCase):
    def _valid_arguments(self):
        return {
            "data": [0, 50, 99, 100],
            "frame_id": "base_link",
            "width": 2,
            "height": 2,
            "resolution_m": 0.1,
            "origin_x_m": 0.0,
            "origin_y_m": -0.1,
            "origin_orientation_xyzw": (0.0, 0.0, 0.0, 1.0),
        }

    def test_weighted_costs_are_retained(self):
        costmap = weighted_costmap_from_grid(**self._valid_arguments())

        np.testing.assert_array_equal(costmap.costs, [[0, 50], [99, 100]])
        self.assertFalse(costmap.costs.flags.writeable)

    def test_invalid_geometry_and_costs_are_rejected(self):
        invalid = (
            {"frame_id": "map"},
            {"width": 0},
            {"data": [0]},
            {"resolution_m": 0.0},
            {"origin_x_m": math.nan},
            {"origin_orientation_xyzw": (0.0, 0.0, 0.1, 0.99)},
            {"data": [0, 50, 99, 101]},
            {"data": [0, 50, -2, 100]},
        )
        for change in invalid:
            arguments = self._valid_arguments()
            arguments.update(change)
            with self.subTest(change=change):
                with self.assertRaises(ValueError):
                    weighted_costmap_from_grid(**arguments)


if __name__ == "__main__":
    unittest.main()
