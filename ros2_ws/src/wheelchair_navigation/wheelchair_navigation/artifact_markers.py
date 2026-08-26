"""Static RViz markers for one L2 artifact box and its XY halo."""

from __future__ import annotations

from geometry_msgs.msg import Point
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

from wheelchair_navigation.artifact_filter import ArtifactBox


def build_artifact_markers(
    header: Header,
    box: ArtifactBox,
    halo_margin_m: float,
    namespace: str,
) -> MarkerArray:
    """Build a translucent hard box and a green halo outline."""

    result = MarkerArray()
    clear = Marker()
    clear.header = header
    clear.action = Marker.DELETEALL
    result.markers.append(clear)

    hard_box = Marker()
    hard_box.header = header
    hard_box.ns = namespace + "/hard_box"
    hard_box.id = 0
    hard_box.type = Marker.CUBE
    hard_box.action = Marker.ADD
    hard_box.pose.position.x = (box.min_x_m + box.max_x_m) / 2.0
    hard_box.pose.position.y = (box.min_y_m + box.max_y_m) / 2.0
    hard_box.pose.position.z = (box.min_z_m + box.max_z_m) / 2.0
    hard_box.pose.orientation.w = 1.0
    hard_box.scale.x = box.max_x_m - box.min_x_m
    hard_box.scale.y = box.max_y_m - box.min_y_m
    hard_box.scale.z = box.max_z_m - box.min_z_m
    hard_box.color.r = 1.0
    hard_box.color.g = 0.12
    hard_box.color.b = 0.12
    hard_box.color.a = 0.28
    hard_box.frame_locked = True
    result.markers.append(hard_box)

    halo = Marker()
    halo.header = header
    halo.ns = namespace + "/halo"
    halo.id = 1
    halo.type = Marker.LINE_LIST
    halo.action = Marker.ADD
    halo.pose.orientation.w = 1.0
    halo.scale.x = 0.018
    halo.color.r = 0.20
    halo.color.g = 1.0
    halo.color.b = 0.20
    halo.color.a = 1.0
    halo.frame_locked = True
    min_x = box.min_x_m - halo_margin_m
    max_x = box.max_x_m + halo_margin_m
    min_y = box.min_y_m - halo_margin_m
    max_y = box.max_y_m + halo_margin_m
    z_m = box.max_z_m + 0.025
    corners = (
        (min_x, min_y),
        (max_x, min_y),
        (max_x, max_y),
        (min_x, max_y),
    )
    for index in range(4):
        for x_m, y_m in (corners[index], corners[(index + 1) % 4]):
            point = Point()
            point.x = x_m
            point.y = y_m
            point.z = z_m
            halo.points.append(point)
    result.markers.append(halo)
    return result


__all__ = ["build_artifact_markers"]
