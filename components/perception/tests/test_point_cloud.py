import struct
import sys
import unittest
from pathlib import Path

import numpy as np
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2, PointField

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from perception.point_cloud import (  # noqa: E402
    quaternion_to_matrix,
    read_xyz_points,
    transform_points,
)


class PointCloudTests(unittest.TestCase):
    def test_reads_organized_cloud_with_row_padding_and_skips_nonfinite(self):
        msg = PointCloud2()
        msg.height = 2
        msg.width = 2
        msg.point_step = 12
        msg.row_step = 28
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.data = (
            struct.pack("<fff", 1.0, 2.0, 3.0)
            + struct.pack("<fff", 4.0, 5.0, 6.0)
            + b"PAD!"
            + struct.pack("<fff", 7.0, 8.0, 9.0)
            + struct.pack("<fff", float("nan"), 0.0, 0.0)
        )

        points = list(read_xyz_points(msg))

        self.assertEqual(points, [(1.0, 2.0, 3.0), (4.0, 5.0, 6.0), (7.0, 8.0, 9.0)])

    def test_rejects_cloud_missing_xyz(self):
        msg = PointCloud2()
        msg.height = 1
        msg.width = 1
        msg.point_step = 4
        msg.row_step = 4
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1)
        ]
        msg.data = struct.pack("<f", 1.0)

        with self.assertRaises(ValueError):
            list(read_xyz_points(msg))

    def test_transforms_points_with_rotation_and_translation(self):
        transform = TransformStamped()
        transform.transform.translation.x = 1.0
        transform.transform.translation.y = 2.0
        transform.transform.translation.z = 3.0
        transform.transform.rotation.z = np.sqrt(0.5)
        transform.transform.rotation.w = np.sqrt(0.5)
        points = np.array([[1.0, 0.0, 0.0]], dtype=np.float32)

        result = transform_points(points, transform)

        np.testing.assert_allclose(result, [[1.0, 3.0, 3.0]], atol=1e-6)

    def test_zero_quaternion_returns_identity(self):
        np.testing.assert_array_equal(
            quaternion_to_matrix(0.0, 0.0, 0.0, 0.0),
            np.eye(3, dtype=np.float32),
        )


if __name__ == "__main__":
    unittest.main()
