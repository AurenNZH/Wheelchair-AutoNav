import math
import unittest

import numpy as np

from wheelchair_shared_control.models import (
    SafetyConfig,
    validate_safety_config,
    weighted_costmap_from_grid,
)


class SafetyConfigValidationTests(unittest.TestCase):
    def test_valid_default_configuration(self):
        validate_safety_config(SafetyConfig())

    def test_invalid_configuration_is_rejected(self):
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
            with self.subTest(config=config), self.assertRaises(ValueError):
                validate_safety_config(config)


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
            with self.subTest(change=change), self.assertRaises(ValueError):
                weighted_costmap_from_grid(**arguments)


if __name__ == "__main__":
    unittest.main()
