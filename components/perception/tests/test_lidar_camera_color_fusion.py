import sys
import unittest
from pathlib import Path

import numpy as np
from sensor_msgs.msg import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from perception.lidar_camera_color_fusion import (  # noqa: E402
    CameraModel,
    colorize_points_with_depth,
    depth_msg_to_meters,
)


def make_depth_message(values: np.ndarray, encoding: str) -> Image:
    msg = Image()
    msg.height, msg.width = values.shape
    msg.encoding = encoding
    msg.is_bigendian = False
    msg.step = values.strides[0]
    msg.data = values.tobytes()
    return msg


class LidarCameraColorFusionTests(unittest.TestCase):
    def setUp(self):
        self.camera = CameraModel(
            frame_id="camera_color_optical_frame",
            fx=1.0,
            fy=1.0,
            cx=1.0,
            cy=1.0,
        )
        self.rgb = np.zeros((3, 3, 3), dtype=np.uint8)
        self.rgb[1, 1] = (10, 20, 30)

    def test_converts_16uc1_millimetres_to_metres(self):
        values = np.array([[0, 500], [1234, 2000]], dtype=np.uint16)

        result = depth_msg_to_meters(make_depth_message(values, "16UC1"))

        np.testing.assert_allclose(result, [[0.0, 0.5], [1.234, 2.0]])
        self.assertEqual(result.dtype, np.float32)

    def test_preserves_32fc1_metres(self):
        values = np.array([[0.25, 1.5]], dtype=np.float32)

        result = depth_msg_to_meters(make_depth_message(values, "32FC1"))

        np.testing.assert_allclose(result, values)

    def test_matching_depth_copies_rgb_and_preserves_xyz(self):
        lidar = np.array([[4.0, 5.0, 6.0]], dtype=np.float32)
        camera = np.array([[0.0, 0.0, 2.0]], dtype=np.float32)
        depth = np.full((3, 3), 2.04, dtype=np.float32)

        output, stats = colorize_points_with_depth(
            lidar, camera, self.rgb, depth, self.camera
        )

        self.assertEqual(output, [(4.0, 5.0, 6.0, 10, 20, 30)])
        self.assertEqual(stats.colored, 1)

    def test_camera_only_hand_rejects_background_lidar_color(self):
        lidar = np.array([[2.0, 0.0, 0.0]], dtype=np.float32)
        camera = np.array([[0.0, 0.0, 2.0]], dtype=np.float32)
        depth = np.full((3, 3), 0.4, dtype=np.float32)

        output, stats = colorize_points_with_depth(
            lidar, camera, self.rgb, depth, self.camera
        )

        self.assertEqual(output, [(2.0, 0.0, 0.0, 128, 128, 128)])
        self.assertEqual(stats.colored, 0)
        self.assertEqual(stats.depth_mismatch, 1)

    def test_invalid_depth_keeps_neutral_point(self):
        lidar = np.array([[1.0, 2.0, 3.0]], dtype=np.float32)
        camera = np.array([[0.0, 0.0, 1.0]], dtype=np.float32)
        depth = np.zeros((3, 3), dtype=np.float32)

        output, stats = colorize_points_with_depth(
            lidar, camera, self.rgb, depth, self.camera
        )

        self.assertEqual(output[0][:3], (1.0, 2.0, 3.0))
        self.assertEqual(output[0][3:], (128, 128, 128))
        self.assertEqual(stats.invalid_depth, 1)

    def test_zbuffer_only_colors_nearest_point_at_pixel(self):
        lidar = np.array([[1.0, 0.0, 0.0], [2.0, 0.0, 0.0]], dtype=np.float32)
        camera = np.array([[0.0, 0.0, 1.0], [0.0, 0.0, 2.0]], dtype=np.float32)
        depth = np.full((3, 3), 1.0, dtype=np.float32)

        output, stats = colorize_points_with_depth(
            lidar, camera, self.rgb, depth, self.camera
        )

        self.assertEqual(output[0][3:], (10, 20, 30))
        self.assertEqual(output[1][3:], (128, 128, 128))
        self.assertEqual(stats.colored, 1)
        self.assertEqual(stats.zbuffer_rejected, 1)

    def test_behind_and_outside_points_are_retained(self):
        lidar = np.array([[1.0, 0.0, 0.0], [2.0, 0.0, 0.0]], dtype=np.float32)
        camera = np.array([[0.0, 0.0, -1.0], [10.0, 0.0, 1.0]], dtype=np.float32)
        depth = np.ones((3, 3), dtype=np.float32)

        output, stats = colorize_points_with_depth(
            lidar, camera, self.rgb, depth, self.camera
        )

        self.assertEqual(len(output), 2)
        self.assertEqual(stats.behind_camera, 1)
        self.assertEqual(stats.outside_image, 1)
        self.assertEqual(stats.colored, 0)


if __name__ == "__main__":
    unittest.main()
