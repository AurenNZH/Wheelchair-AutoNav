import unittest

from wheelchair_shared_control.operator_intent import (
    FORWARD,
    FORWARD_LEFT,
    FORWARD_RIGHT,
    LEFT_TURN,
    RELEASED,
    REVERSE,
    REVERSE_LEFT,
    REVERSE_RIGHT,
    RIGHT_TURN,
    classify_normalized_axes,
    is_valid_hard_turn,
)


class OperatorIntentClassificationTests(unittest.TestCase):
    def test_recorded_pi_vectors_match_ros_classes(self):
        # Pi X is right-positive, so ROS lateral is its negation.
        fixtures = {
            (0.0, 0.0): RELEASED,
            (0.0, 1.0): FORWARD,
            (0.14, 0.99): FORWARD_LEFT,
            (-0.20, 1.0): FORWARD_RIGHT,
            (0.96, 0.12): LEFT_TURN,
            (-1.0, 0.0): RIGHT_TURN,
            (-0.98, -0.20): RIGHT_TURN,
            (0.0, -1.0): REVERSE,
            (0.20, -1.0): REVERSE_LEFT,
            (-0.20, -1.0): REVERSE_RIGHT,
        }
        for axes, expected in fixtures.items():
            with self.subTest(axes=axes):
                self.assertEqual(
                    classify_normalized_axes(*axes).intent_class,
                    expected,
                )

    def test_neutral_and_cone_boundaries_are_inclusive(self):
        self.assertEqual(
            classify_normalized_axes(0.05, -0.05).intent_class,
            RELEASED,
        )
        self.assertTrue(
            classify_normalized_axes(0.57, 1.0).is_forward
        )
        self.assertEqual(
            classify_normalized_axes(0.58, 1.0).intent_class,
            LEFT_TURN,
        )
        self.assertTrue(
            classify_normalized_axes(-0.57, 1.0).is_forward
        )
        self.assertEqual(
            classify_normalized_axes(-0.58, 1.0).intent_class,
            RIGHT_TURN,
        )
        self.assertTrue(
            classify_normalized_axes(0.57, -1.0).is_reverse
        )
        self.assertTrue(
            classify_normalized_axes(-0.57, -1.0).is_reverse
        )

    def test_reverse_corrections_report_signed_steering_ratio(self):
        left = classify_normalized_axes(0.2, -1.0)
        right = classify_normalized_axes(-0.2, -1.0)

        self.assertTrue(left.is_reverse)
        self.assertTrue(right.is_reverse)
        self.assertAlmostEqual(left.steering_ratio, 0.2)
        self.assertAlmostEqual(right.steering_ratio, -0.2)

    def test_only_consistent_active_hard_turns_request_disc_support(self):
        self.assertTrue(is_valid_hard_turn(0.8, 0.0, LEFT_TURN, True))
        self.assertTrue(is_valid_hard_turn(-0.8, 0.0, RIGHT_TURN, True))
        self.assertFalse(is_valid_hard_turn(0.0, 0.8, FORWARD, True))
        self.assertFalse(is_valid_hard_turn(0.8, 0.0, FORWARD_LEFT, True))
        self.assertFalse(is_valid_hard_turn(0.8, 0.0, LEFT_TURN, False))


if __name__ == "__main__":
    unittest.main()
