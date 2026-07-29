import struct
import unittest

import numpy as np
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2, PointField

from wheelchair_navigation.point_cloud import (
    point_cloud_to_arrays,
    quaternion_to_matrix,
    transform_points,
)


class PointCloudTests(unittest.TestCase):
    def test_vectorized_decode_respects_organized_row_padding(self):
        msg = PointCloud2()
        msg.height = 2
        msg.width = 2
        msg.point_step = 12
        msg.row_step = 28
        msg.fields = self._xyz_fields()
        msg.data = (
            struct.pack("<fff", 1.0, 2.0, 3.0)
            + struct.pack("<fff", 4.0, 5.0, 6.0)
            + b"PAD!"
            + struct.pack("<fff", 7.0, 8.0, 9.0)
            + struct.pack("<fff", float("nan"), 0.0, 0.0)
        )

        cloud = point_cloud_to_arrays(msg)

        np.testing.assert_allclose(
            cloud.xyz[:3],
            [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]],
        )
        self.assertTrue(np.isnan(cloud.xyz[3, 0]))

    def test_decodes_airy_diagnostic_fields(self):
        msg = PointCloud2()
        msg.height = 1
        msg.width = 1
        msg.point_step = 30
        msg.row_step = 30
        msg.fields = self._xyz_fields() + [
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name="ring", offset=16, datatype=PointField.UINT16, count=1),
            PointField(name="timestamp", offset=22, datatype=PointField.FLOAT64, count=1),
        ]
        data = bytearray(30)
        struct.pack_into("<fff", data, 0, 1.0, 2.0, 3.0)
        struct.pack_into("<f", data, 12, 42.5)
        struct.pack_into("<H", data, 16, 17)
        struct.pack_into("<d", data, 22, 123.25)
        msg.data = bytes(data)

        cloud = point_cloud_to_arrays(msg)

        np.testing.assert_allclose(cloud.intensity, [42.5])
        np.testing.assert_array_equal(cloud.ring, [17])
        np.testing.assert_allclose(cloud.timestamp, [123.25])

    def test_decodes_big_endian_xyz(self):
        msg = PointCloud2()
        msg.height = 1
        msg.width = 1
        msg.point_step = 12
        msg.row_step = 12
        msg.is_bigendian = True
        msg.fields = self._xyz_fields()
        msg.data = struct.pack(">fff", 1.0, 2.0, 3.0)

        np.testing.assert_allclose(point_cloud_to_arrays(msg).xyz, [[1.0, 2.0, 3.0]])

    def test_rejects_missing_or_truncated_fields(self):
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
            point_cloud_to_arrays(msg)

        msg.fields = self._xyz_fields()
        msg.point_step = 12
        msg.row_step = 12
        with self.assertRaises(ValueError):
            point_cloud_to_arrays(msg)

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

    def test_calibrated_native_minus_y_maps_to_robot_forward_plus_x(self):
        transform = TransformStamped()
        transform.transform.rotation.z = np.sqrt(0.5)
        transform.transform.rotation.w = np.sqrt(0.5)
        native_forward = np.array([[0.0, -2.0, 0.0]], dtype=np.float32)

        result = transform_points(native_forward, transform)

        np.testing.assert_allclose(result, [[2.0, 0.0, 0.0]], atol=1e-6)

    def test_zero_quaternion_returns_identity(self):
        np.testing.assert_array_equal(
            quaternion_to_matrix(0.0, 0.0, 0.0, 0.0),
            np.eye(3, dtype=np.float32),
        )

    @staticmethod
    def _xyz_fields():
        return [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]


if __name__ == "__main__":
    unittest.main()
