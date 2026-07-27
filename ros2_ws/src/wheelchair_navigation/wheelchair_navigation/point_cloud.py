"""Vectorized PointCloud2 decoding and rigid-transform utilities."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField


@dataclass(frozen=True)
class PointCloudArrays:
    xyz: np.ndarray
    intensity: np.ndarray | None = None
    ring: np.ndarray | None = None
    timestamp: np.ndarray | None = None


_FIELD_DTYPES = {
    PointField.INT8: "i1",
    PointField.UINT8: "u1",
    PointField.INT16: "i2",
    PointField.UINT16: "u2",
    PointField.INT32: "i4",
    PointField.UINT32: "u4",
    PointField.FLOAT32: "f4",
    PointField.FLOAT64: "f8",
}


def point_cloud_to_arrays(msg: PointCloud2) -> PointCloudArrays:
    """Decode supported fields without a Python loop over individual points."""

    fields = {field.name: field for field in msg.fields}
    missing = {"x", "y", "z"} - set(fields)
    if missing:
        raise ValueError("PointCloud2 missing XYZ fields: %s" % sorted(missing))
    if msg.point_step <= 0:
        raise ValueError("PointCloud2 point_step must be positive.")

    byte_order = ">" if msg.is_bigendian else "<"
    names = []
    formats = []
    offsets = []
    for name in ("x", "y", "z", "intensity", "ring", "timestamp"):
        field = fields.get(name)
        if field is None:
            continue
        if field.count != 1 or field.datatype not in _FIELD_DTYPES:
            raise ValueError("Unsupported PointCloud2 field: %s" % name)
        item_dtype = np.dtype(byte_order + _FIELD_DTYPES[field.datatype])
        if field.offset < 0 or field.offset + item_dtype.itemsize > msg.point_step:
            raise ValueError("PointCloud2 %s field lies outside point_step." % name)
        names.append(name)
        formats.append(item_dtype)
        offsets.append(field.offset)

    for name in ("x", "y", "z"):
        if fields[name].datatype != PointField.FLOAT32 or fields[name].count != 1:
            raise ValueError("PointCloud2 XYZ fields must be scalar FLOAT32 values.")

    dtype = np.dtype(
        {"names": names, "formats": formats, "offsets": offsets, "itemsize": msg.point_step}
    )
    height = max(1, int(msg.height))
    width = int(msg.width)
    row_step = int(msg.row_step) or width * int(msg.point_step)
    if row_step < width * int(msg.point_step):
        raise ValueError("PointCloud2 row_step is shorter than one row.")
    required_bytes = (height - 1) * row_step + width * int(msg.point_step)
    if len(msg.data) < required_bytes:
        raise ValueError("PointCloud2 data is shorter than its declared dimensions.")

    structured = np.ndarray(
        shape=(height, width),
        dtype=dtype,
        buffer=msg.data,
        strides=(row_step, int(msg.point_step)),
    )
    xyz = np.column_stack(
        (
            structured["x"].reshape(-1),
            structured["y"].reshape(-1),
            structured["z"].reshape(-1),
        )
    ).astype(np.float32, copy=False)

    return PointCloudArrays(
        xyz=xyz,
        intensity=_optional_flat(structured, "intensity"),
        ring=_optional_flat(structured, "ring"),
        timestamp=_optional_flat(structured, "timestamp"),
    )


def _optional_flat(structured: np.ndarray, name: str) -> np.ndarray | None:
    if name not in structured.dtype.names:
        return None
    return np.asarray(structured[name]).reshape(-1)


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
    xx, yy, zz = x * x * scale, y * y * scale, z * z * scale
    xy, xz, yz = x * y * scale, x * z * scale, y * z * scale
    wx, wy, wz = w * x * scale, w * y * scale, w * z * scale
    return np.array(
        [
            [1.0 - yy - zz, xy - wz, xz + wy],
            [xy + wz, 1.0 - xx - zz, yz - wx],
            [xz - wy, yz + wx, 1.0 - xx - yy],
        ],
        dtype=np.float32,
    )


__all__ = [
    "PointCloudArrays",
    "point_cloud_to_arrays",
    "quaternion_to_matrix",
    "transform_points",
]
