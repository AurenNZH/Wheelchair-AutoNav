"""Depth-checked colorization of LiDAR points with an RGB-D camera."""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass
from typing import Iterable

import numpy as np
import rclpy
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener


@dataclass(frozen=True)
class CameraModel:
    frame_id: str
    fx: float
    fy: float
    cx: float
    cy: float


@dataclass(frozen=True)
class FusionStats:
    total: int
    colored: int
    behind_camera: int
    outside_image: int
    invalid_depth: int
    depth_mismatch: int
    zbuffer_rejected: int


class LidarCameraColorFusionNode(Node):
    """Publish a camera-colored LiDAR PointCloud2.

    RGB, aligned depth, and LiDAR messages are approximately synchronized.
    Points are transformed into the color optical frame and receive image
    color only when the RealSense depth and projected LiDAR depth agree.
    Unassociated points remain in the output with a neutral color by default.
    """

    def __init__(self) -> None:
        super().__init__("lidar_camera_color_fusion")

        self.declare_parameter("lidar_topic", "/rslidar_points")
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter(
            "depth_topic", "/camera/aligned_depth_to_color/image_raw"
        )
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("output_topic", "/rslidar_points_colored")
        self.declare_parameter("camera_frame", "")
        self.declare_parameter("keep_uncolored", True)
        self.declare_parameter("uncolored_rgb", [128, 128, 128])
        self.declare_parameter("max_points", 0)
        self.declare_parameter("sync_queue_size", 10)
        self.declare_parameter("sync_slop_sec", 0.08)
        self.declare_parameter("depth_tolerance_min_m", 0.08)
        self.declare_parameter("depth_tolerance_ratio", 0.03)

        self._camera_model: CameraModel | None = None

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        lidar_topic = self.get_parameter("lidar_topic").value
        image_topic = self.get_parameter("image_topic").value
        depth_topic = self.get_parameter("depth_topic").value
        camera_info_topic = self.get_parameter("camera_info_topic").value
        output_topic = self.get_parameter("output_topic").value

        self._pub = self.create_publisher(PointCloud2, output_topic, 10)
        self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self._on_camera_info,
            qos_profile_sensor_data,
        )

        self._image_sub = Subscriber(
            self, Image, image_topic, qos_profile=qos_profile_sensor_data
        )
        self._depth_sub = Subscriber(
            self, Image, depth_topic, qos_profile=qos_profile_sensor_data
        )
        self._lidar_sub = Subscriber(
            self, PointCloud2, lidar_topic, qos_profile=qos_profile_sensor_data
        )
        self._synchronizer = ApproximateTimeSynchronizer(
            [self._image_sub, self._depth_sub, self._lidar_sub],
            queue_size=int(self.get_parameter("sync_queue_size").value),
            slop=float(self.get_parameter("sync_slop_sec").value),
        )
        self._synchronizer.registerCallback(self._on_synced_data)

        self.get_logger().info(
            "Depth-checked colorization: %s + %s + %s -> %s"
            % (lidar_topic, image_topic, depth_topic, output_topic)
        )

    def _on_camera_info(self, msg: CameraInfo) -> None:
        frame_id = self.get_parameter("camera_frame").value or msg.header.frame_id
        self._camera_model = CameraModel(
            frame_id=frame_id,
            fx=float(msg.k[0]),
            fy=float(msg.k[4]),
            cx=float(msg.k[2]),
            cy=float(msg.k[5]),
        )

    def _on_synced_data(
        self, image_msg: Image, depth_msg: Image, lidar_msg: PointCloud2
    ) -> None:
        if self._camera_model is None:
            self.get_logger().warn(
                "Waiting for camera_info before colorizing.",
                throttle_duration_sec=5.0,
            )
            return

        try:
            image_rgb = image_msg_to_rgb(image_msg)
            depth_m = depth_msg_to_meters(depth_msg)
        except ValueError as exc:
            self.get_logger().warn(str(exc), throttle_duration_sec=5.0)
            return

        if depth_m.shape != image_rgb.shape[:2]:
            self.get_logger().warn(
                "Aligned depth shape %s does not match RGB shape %s."
                % (depth_m.shape, image_rgb.shape[:2]),
                throttle_duration_sec=5.0,
            )
            return

        try:
            transform = self._tf_buffer.lookup_transform(
                self._camera_model.frame_id,
                lidar_msg.header.frame_id,
                Time.from_msg(lidar_msg.header.stamp),
            )
        except TransformException as exc:
            self.get_logger().warn(
                "No timestamped TF from %s to %s: %s"
                % (lidar_msg.header.frame_id, self._camera_model.frame_id, exc),
                throttle_duration_sec=5.0,
            )
            return

        points = np.asarray(list(read_xyz_points(lidar_msg)), dtype=np.float32)
        if points.size == 0:
            return

        max_points = int(self.get_parameter("max_points").value)
        if max_points > 0 and points.shape[0] > max_points:
            step = int(math.ceil(points.shape[0] / max_points))
            points = points[::step]

        points_camera = transform_points(points, transform)
        colored, stats = colorize_points_with_depth(
            points,
            points_camera,
            image_rgb,
            depth_m,
            self._camera_model,
            keep_uncolored=bool(self.get_parameter("keep_uncolored").value),
            uncolored_rgb=tuple(int(v) for v in self.get_parameter("uncolored_rgb").value),
            depth_tolerance_min_m=float(
                self.get_parameter("depth_tolerance_min_m").value
            ),
            depth_tolerance_ratio=float(
                self.get_parameter("depth_tolerance_ratio").value
            ),
        )
        if not colored:
            self.get_logger().warn(
                "No LiDAR points projected into the camera image.",
                throttle_duration_sec=5.0,
            )
            return

        out = build_colored_cloud(lidar_msg.header, colored)
        self._pub.publish(out)
        self.get_logger().info(
            "fusion total=%d colored=%d mismatch=%d invalid_depth=%d "
            "outside=%d behind=%d zbuffer=%d"
            % (
                stats.total,
                stats.colored,
                stats.depth_mismatch,
                stats.invalid_depth,
                stats.outside_image,
                stats.behind_camera,
                stats.zbuffer_rejected,
            ),
            throttle_duration_sec=2.0,
        )


def image_msg_to_rgb(msg: Image) -> np.ndarray:
    """Convert common ROS Image encodings to an RGB numpy array."""

    encoding = msg.encoding.lower()
    channels_by_encoding = {
        "rgb8": 3,
        "bgr8": 3,
        "rgba8": 4,
        "bgra8": 4,
        "mono8": 1,
    }
    if encoding not in channels_by_encoding:
        raise ValueError("Unsupported image encoding for color fusion: %s" % msg.encoding)

    channels = channels_by_encoding[encoding]
    row = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    pixels = row[:, : msg.width * channels].reshape(msg.height, msg.width, channels)

    if encoding == "rgb8":
        return pixels.copy()
    if encoding == "bgr8":
        return pixels[:, :, ::-1].copy()
    if encoding == "rgba8":
        return pixels[:, :, :3].copy()
    if encoding == "bgra8":
        return pixels[:, :, 2::-1].copy()

    return np.repeat(pixels, 3, axis=2)


def depth_msg_to_meters(msg: Image) -> np.ndarray:
    """Convert aligned RealSense depth to a float32 image in metres."""

    encoding = msg.encoding.lower()
    if encoding in {"16uc1", "mono16"}:
        dtype = np.dtype(">u2" if msg.is_bigendian else "<u2")
        scale = 0.001
    elif encoding == "32fc1":
        dtype = np.dtype(">f4" if msg.is_bigendian else "<f4")
        scale = 1.0
    else:
        raise ValueError("Unsupported aligned depth encoding: %s" % msg.encoding)

    if msg.step % dtype.itemsize != 0:
        raise ValueError("Depth image step is not aligned to its element size.")

    row_elements = msg.step // dtype.itemsize
    values = np.frombuffer(msg.data, dtype=dtype)
    expected = msg.height * row_elements
    if values.size < expected or row_elements < msg.width:
        raise ValueError("Depth image data is shorter than its declared dimensions.")

    image = values[:expected].reshape(msg.height, row_elements)[:, : msg.width]
    return image.astype(np.float32) * scale


def read_xyz_points(msg: PointCloud2) -> Iterable[tuple[float, float, float]]:
    offsets = {field.name: field.offset for field in msg.fields}
    missing = {"x", "y", "z"} - set(offsets)
    if missing:
        raise ValueError("PointCloud2 missing XYZ fields: %s" % sorted(missing))

    endian = ">" if msg.is_bigendian else "<"
    unpack_float = struct.Struct("%sf" % endian).unpack_from
    data = memoryview(msg.data)

    for point_offset in range(0, len(msg.data), msg.point_step):
        x = unpack_float(data, point_offset + offsets["x"])[0]
        y = unpack_float(data, point_offset + offsets["y"])[0]
        z = unpack_float(data, point_offset + offsets["z"])[0]
        if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
            yield x, y, z


def transform_points(points: np.ndarray, transform) -> np.ndarray:
    t = transform.transform.translation
    q = transform.transform.rotation
    rotation = quaternion_to_matrix(q.x, q.y, q.z, q.w)
    translation = np.array([t.x, t.y, t.z], dtype=np.float32)
    return points @ rotation.T + translation


def quaternion_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
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


def colorize_points(
    lidar_points: np.ndarray,
    camera_points: np.ndarray,
    image_rgb: np.ndarray,
    camera: CameraModel,
    keep_uncolored: bool = False,
    uncolored_rgb: tuple[int, int, int] = (128, 128, 128),
) -> list[tuple[float, float, float, int, int, int]]:
    height, width = image_rgb.shape[:2]
    output: list[tuple[float, float, float, int, int, int]] = []

    for lidar_point, camera_point in zip(lidar_points, camera_points):
        x_cam, y_cam, z_cam = camera_point
        rgb = None
        if z_cam > 0.0:
            u = int(round((camera.fx * x_cam / z_cam) + camera.cx))
            v = int(round((camera.fy * y_cam / z_cam) + camera.cy))
            if 0 <= u < width and 0 <= v < height:
                rgb = tuple(int(channel) for channel in image_rgb[v, u])

        if rgb is None:
            if not keep_uncolored:
                continue
            rgb = uncolored_rgb

        x_lidar, y_lidar, z_lidar = lidar_point
        output.append((float(x_lidar), float(y_lidar), float(z_lidar), rgb[0], rgb[1], rgb[2]))

    return output


def colorize_points_with_depth(
    lidar_points: np.ndarray,
    camera_points: np.ndarray,
    image_rgb: np.ndarray,
    depth_m: np.ndarray,
    camera: CameraModel,
    keep_uncolored: bool = True,
    uncolored_rgb: tuple[int, int, int] = (128, 128, 128),
    depth_tolerance_min_m: float = 0.08,
    depth_tolerance_ratio: float = 0.03,
) -> tuple[list[tuple[float, float, float, int, int, int]], FusionStats]:
    """Associate color only where projected LiDAR and RGB-D depths agree.

    A z-buffer prevents multiple LiDAR returns projected to the same pixel from
    all inheriting that pixel's color. Points that fail association are retained
    with ``uncolored_rgb`` when ``keep_uncolored`` is true.
    """

    if lidar_points.shape != camera_points.shape or lidar_points.ndim != 2:
        raise ValueError("LiDAR and camera points must have matching (N, 3) shapes.")
    if lidar_points.shape[1] != 3:
        raise ValueError("LiDAR and camera points must have matching (N, 3) shapes.")
    if depth_m.shape != image_rgb.shape[:2]:
        raise ValueError("Aligned depth and RGB dimensions must match.")
    if depth_tolerance_min_m < 0.0 or depth_tolerance_ratio < 0.0:
        raise ValueError("Depth tolerances must be non-negative.")

    total = lidar_points.shape[0]
    height, width = depth_m.shape
    colors = np.tile(np.asarray(uncolored_rgb, dtype=np.uint8), (total, 1))
    associated = np.zeros(total, dtype=bool)

    behind_camera = 0
    outside_image = 0
    invalid_depth = 0
    depth_mismatch = 0
    zbuffer_rejected = 0
    nearest_by_pixel: dict[tuple[int, int], tuple[float, int]] = {}

    for index, camera_point in enumerate(camera_points):
        x_cam, y_cam, z_cam = (float(value) for value in camera_point)
        if not math.isfinite(z_cam) or z_cam <= 0.0:
            behind_camera += 1
            continue

        u = int(round((camera.fx * x_cam / z_cam) + camera.cx))
        v = int(round((camera.fy * y_cam / z_cam) + camera.cy))
        if not (0 <= u < width and 0 <= v < height):
            outside_image += 1
            continue

        key = (u, v)
        previous = nearest_by_pixel.get(key)
        if previous is None or z_cam < previous[0]:
            if previous is not None:
                zbuffer_rejected += 1
            nearest_by_pixel[key] = (z_cam, index)
        else:
            zbuffer_rejected += 1

    for (u, v), (lidar_depth, index) in nearest_by_pixel.items():
        camera_depth = float(depth_m[v, u])
        if not math.isfinite(camera_depth) or camera_depth <= 0.0:
            invalid_depth += 1
            continue

        tolerance = max(
            depth_tolerance_min_m,
            depth_tolerance_ratio * min(camera_depth, lidar_depth),
        )
        if abs(camera_depth - lidar_depth) > tolerance:
            depth_mismatch += 1
            continue

        colors[index] = image_rgb[v, u, :3]
        associated[index] = True

    if keep_uncolored:
        selected_indices = range(total)
    else:
        selected_indices = np.flatnonzero(associated)

    output = [
        (
            float(lidar_points[index, 0]),
            float(lidar_points[index, 1]),
            float(lidar_points[index, 2]),
            int(colors[index, 0]),
            int(colors[index, 1]),
            int(colors[index, 2]),
        )
        for index in selected_indices
    ]
    stats = FusionStats(
        total=total,
        colored=int(np.count_nonzero(associated)),
        behind_camera=behind_camera,
        outside_image=outside_image,
        invalid_depth=invalid_depth,
        depth_mismatch=depth_mismatch,
        zbuffer_rejected=zbuffer_rejected,
    )
    return output, stats


def build_colored_cloud(
    header: Header,
    points: list[tuple[float, float, float, int, int, int]],
) -> PointCloud2:
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = len(points)
    msg.is_bigendian = False
    msg.is_dense = True
    msg.point_step = 16
    msg.row_step = msg.point_step * msg.width
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
    ]

    packed = bytearray()
    pack = struct.Struct("<fffI").pack
    for x, y, z, r, g, b in points:
        rgb_uint32 = (int(r) << 16) | (int(g) << 8) | int(b)
        packed.extend(pack(x, y, z, rgb_uint32))
    msg.data = bytes(packed)
    return msg


def main() -> int:
    rclpy.init()
    node = LidarCameraColorFusionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
