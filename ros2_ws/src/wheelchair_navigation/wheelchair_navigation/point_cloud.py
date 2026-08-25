"""Vectorized PointCloud2 decoding and rigid-transform utilities."""

from __future__ import annotations

from array import array
from dataclasses import dataclass

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


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


def xyz_to_point_cloud(points: np.ndarray, header: Header) -> PointCloud2:
    """Build a compact XYZ cloud while preserving the supplied header."""

    xyz = np.asarray(points, dtype=np.float32)
    if xyz.size == 0:
        xyz = np.empty((0, 3), dtype=np.float32)
    if xyz.ndim != 2 or xyz.shape[1] != 3:
        raise ValueError("points must have shape (N, 3)")
    little_endian = xyz.astype("<f4", copy=False)

    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = int(xyz.shape[0])
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = msg.width * msg.point_step
    msg.data = little_endian.tobytes(order="C")
    msg.is_dense = bool(np.isfinite(xyz).all())
    return msg


def select_point_cloud_records(
    msg: PointCloud2, keep_mask: np.ndarray
) -> PointCloud2:
    """Select complete point records while preserving the input schema.

    The result is compacted to an unorganized cloud because removing arbitrary
    points cannot preserve the source image layout. Every byte in each retained
    point record is copied unchanged, including vendor fields that the
    filtering algorithm does not interpret.
    """

    if msg.point_step <= 0:
        raise ValueError("PointCloud2 point_step must be positive.")
    height = max(1, int(msg.height))
    width = int(msg.width)
    if width < 0:
        raise ValueError("PointCloud2 width must be non-negative.")
    row_step = int(msg.row_step) or width * int(msg.point_step)
    packed_row_step = width * int(msg.point_step)
    if row_step < packed_row_step:
        raise ValueError("PointCloud2 row_step is shorter than one row.")
    required_bytes = (height - 1) * row_step + packed_row_step
    if len(msg.data) < required_bytes:
        raise ValueError("PointCloud2 data is shorter than its declared dimensions.")

    keep = np.asarray(keep_mask, dtype=bool)
    if keep.shape != (height * width,):
        raise ValueError("keep_mask must have shape (height * width,)")
    record_dtype = np.dtype((np.void, int(msg.point_step)))
    records = np.ndarray(
        shape=(height, width),
        dtype=record_dtype,
        buffer=msg.data,
        strides=(row_step, int(msg.point_step)),
    ).reshape(-1)
    selected_data = records[keep].tobytes(order="C")

    result = PointCloud2()
    result.header = msg.header
    result.height = 1
    result.width = int(np.count_nonzero(keep))
    result.fields = list(msg.fields)
    result.is_bigendian = msg.is_bigendian
    result.point_step = int(msg.point_step)
    result.row_step = result.width * result.point_step
    packed_data = array("B")
    packed_data.frombytes(selected_data)
    # Assigning bytes to Foxy's generated uint8[] setter performs a slow
    # Python-level conversion. Supplying the native array avoids that second
    # per-byte pass while preserving exactly the same serialized payload.
    result.data = packed_data
    # Preserve the conservative source claim. The filter node removes invalid
    # XYZ records, but this generic helper does not interpret arbitrary fields.
    result.is_dense = msg.is_dense
    return result


__all__ = [
    "PointCloudArrays",
    "point_cloud_to_arrays",
    "quaternion_to_matrix",
    "select_point_cloud_records",
    "transform_points",
    "xyz_to_point_cloud",
]
