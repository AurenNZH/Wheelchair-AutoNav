"""PointCloud2 parsing and rigid-transform utilities for navigation."""

from __future__ import annotations

import math
import struct
from typing import Iterable

import numpy as np
from sensor_msgs.msg import PointCloud2


def read_xyz_points(msg: PointCloud2) -> Iterable[tuple[float, float, float]]:
    """Yield finite XYZ points while respecting organized-cloud row padding."""

    offsets = {field.name: field.offset for field in msg.fields}
    missing = {"x", "y", "z"} - set(offsets)
    if missing:
        raise ValueError("PointCloud2 missing XYZ fields: %s" % sorted(missing))
    if msg.point_step <= 0:
        raise ValueError("PointCloud2 point_step must be positive.")

    for name in ("x", "y", "z"):
        if offsets[name] < 0 or offsets[name] + 4 > msg.point_step:
            raise ValueError("PointCloud2 %s field lies outside point_step." % name)

    endian = ">" if msg.is_bigendian else "<"
    unpack_float = struct.Struct("%sf" % endian).unpack_from
    data = memoryview(msg.data)
    height = max(1, int(msg.height))
    width = int(msg.width)
    row_step = int(msg.row_step) or width * int(msg.point_step)
    required_bytes = (height - 1) * row_step + width * int(msg.point_step)
    if len(data) < required_bytes:
        raise ValueError("PointCloud2 data is shorter than its declared dimensions.")

    for row in range(height):
        row_offset = row * row_step
        for column in range(width):
            point_offset = row_offset + column * msg.point_step
            x = unpack_float(data, point_offset + offsets["x"])[0]
            y = unpack_float(data, point_offset + offsets["y"])[0]
            z = unpack_float(data, point_offset + offsets["z"])[0]
            if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
                yield x, y, z


def transform_points(points: np.ndarray, transform) -> np.ndarray:
    """Apply a geometry_msgs TransformStamped-compatible rigid transform."""

    t = transform.transform.translation
    q = transform.transform.rotation
    rotation = quaternion_to_matrix(q.x, q.y, q.z, q.w)
    translation = np.array([t.x, t.y, t.z], dtype=np.float32)
    return np.asarray(points, dtype=np.float32) @ rotation.T + translation


def quaternion_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    """Return a 3x3 rotation matrix for a quaternion."""

    norm = x * x + y * y + z * z + w * w
    if norm < 1e-12:
        return np.eye(3, dtype=np.float32)

    scale = 2.0 / norm
    xx = x * x * scale
    yy = y * y * scale
    zz = z * z * scale
    xy = x * y * scale
    xz = x * z * scale
    yz = y * z * scale
    wx = w * x * scale
    wy = w * y * scale
    wz = w * z * scale

    return np.array(
        [
            [1.0 - yy - zz, xy - wz, xz + wy],
            [xy + wz, 1.0 - xx - zz, yz - wx],
            [xz - wy, yz + wx, 1.0 - xx - yy],
        ],
        dtype=np.float32,
    )


__all__ = ["quaternion_to_matrix", "read_xyz_points", "transform_points"]
