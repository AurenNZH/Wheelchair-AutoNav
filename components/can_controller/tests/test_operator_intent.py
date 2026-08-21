from pathlib import Path
import sys
import unittest


COMPONENT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(COMPONENT / "src"))

from wheelchair_teleop.operator_intent import (
    FORWARD,
    FORWARD_LEFT,
    FORWARD_RIGHT,
    LEFT_TURN,
    RELEASED,
    REVERSE,
    REVERSE_LEFT,
    REVERSE_RIGHT,
    RIGHT_TURN,
    classify_raw_axes,
)


class OperatorIntentClassificationTests(unittest.TestCase):
    def test_recorded_calibration_vectors_have_expected_classes(self):
        fixtures = {
            (0, 0): RELEASED,
            (0, 100): FORWARD,
            (-14, 99): FORWARD_LEFT,
            (20, 100): FORWARD_RIGHT,
            (-96, 12): LEFT_TURN,
            (100, 0): RIGHT_TURN,
            (98, -20): RIGHT_TURN,
            (0, -100): REVERSE,
            (-20, -100): REVERSE_LEFT,
            (20, -100): REVERSE_RIGHT,
        }
        for axes, expected in fixtures.items():
            with self.subTest(axes=axes):
                self.assertEqual(
                    classify_raw_axes(*axes).intent_class,
                    expected,
                )

    def test_release_uses_both_axes_and_preserves_normalized_values(self):
        released = classify_raw_axes(5, -5)
        lateral = classify_raw_axes(6, 0)

        self.assertEqual(released.intent_class, RELEASED)
        self.assertFalse(released.deadman)
        self.assertEqual(lateral.intent_class, RIGHT_TURN)
        self.assertTrue(lateral.deadman)
        self.assertAlmostEqual(lateral.lateral, -0.06)

    def test_forward_cone_boundary_is_inclusive(self):
        # tan(30 degrees) * 100 is about 57.7 raw counts.
        for x_raw in (-57, 57):
            self.assertTrue(classify_raw_axes(x_raw, 100).is_forward)
            self.assertTrue(classify_raw_axes(x_raw, -100).is_reverse)
        self.assertEqual(classify_raw_axes(-58, 100).intent_class, LEFT_TURN)
        self.assertEqual(classify_raw_axes(58, 100).intent_class, RIGHT_TURN)
        self.assertEqual(classify_raw_axes(-58, -100).intent_class, LEFT_TURN)
        self.assertEqual(classify_raw_axes(58, -100).intent_class, RIGHT_TURN)

    def test_forward_steering_ratio_uses_vector_direction(self):
        correction = classify_raw_axes(-14, 99)
        self.assertAlmostEqual(correction.steering_ratio, 14.0 / 99.0)

    def test_reverse_steering_ratio_preserves_raw_correction_direction(self):
        left = classify_raw_axes(-20, -100)
        right = classify_raw_axes(20, -100)

        self.assertTrue(left.is_reverse)
        self.assertTrue(right.is_reverse)
        self.assertAlmostEqual(left.steering_ratio, 0.2)
        self.assertAlmostEqual(right.steering_ratio, -0.2)


if __name__ == "__main__":
    unittest.main()
