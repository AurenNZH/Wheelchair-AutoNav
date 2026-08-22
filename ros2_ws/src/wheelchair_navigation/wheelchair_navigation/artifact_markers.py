"""Visualization builders for calibrated artifact-filter geometry."""

from __future__ import annotations

import numpy as np
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

from wheelchair_navigation.artifact_filter import (
    ArtifactGridCell,
    ArtifactHaloSpan,
    artifact_configured_halo_cell_ids,
)
from wheelchair_navigation.costmap import FrontCostmapConfig


def build_artifact_grid_markers(
    header: Header,
    cells: tuple[ArtifactGridCell, ...],
    halo_spans: tuple[ArtifactHaloSpan, ...],
    config: FrontCostmapConfig,
) -> MarkerArray:
    """Visualize consolidated cell meshes and their staircase halo outlines."""

    result = MarkerArray()
    clear = Marker()
    clear.header = header
    clear.action = Marker.DELETEALL
    result.markers.append(clear)
    colors = (
        (0.10, 0.85, 1.00),
        (0.85, 0.20, 1.00),
        (1.00, 0.55, 0.05),
    )
    resolution = float(config.resolution_m)
    region_ids = sorted({cell.region_id for cell in cells})
    for region_id in region_ids:
        region_cells = tuple(
            cell for cell in cells if cell.region_id == region_id
        )
        red, green, blue = colors[region_id % len(colors)]
        mesh = Marker()
        mesh.header = header
        mesh.ns = "artifact_grid_regions"
        mesh.id = region_id
        mesh.type = Marker.TRIANGLE_LIST
        mesh.action = Marker.ADD
        mesh.pose.orientation.w = 1.0
        mesh.color.r = red
        mesh.color.g = green
        mesh.color.b = blue
        mesh.color.a = 0.34
        mesh.frame_locked = True
        for cell in region_cells:
            mesh.points.extend(_cell_box_triangle_points(cell, resolution))
        result.markers.append(mesh)

        outline = Marker()
        outline.header = header
        outline.ns = "artifact_grid_region_outlines"
        outline.id = region_id
        outline.type = Marker.LINE_LIST
        outline.action = Marker.ADD
        outline.pose.orientation.w = 1.0
        outline.scale.x = 0.012
        outline.color.r = red
        outline.color.g = green
        outline.color.b = blue
        outline.color.a = 1.0
        outline.frame_locked = True
        outline.points = _cell_footprint_outline_points(
            {
                (cell.forward_cell, cell.lateral_cell)
                for cell in region_cells
            },
            resolution,
            max(cell.max_z_m for cell in region_cells) + 0.006,
        )
        result.markers.append(outline)

        halo_outline = Marker()
        halo_outline.header = header
        halo_outline.ns = "artifact_grid_halo_outlines"
        halo_outline.id = region_id
        halo_outline.type = Marker.LINE_LIST
        halo_outline.action = Marker.ADD
        halo_outline.pose.orientation.w = 1.0
        halo_outline.scale.x = 0.018
        halo_outline.color.r = 0.25
        halo_outline.color.g = 1.0
        halo_outline.color.b = 0.25
        halo_outline.color.a = 1.0
        halo_outline.frame_locked = True
        halo_ids = artifact_configured_halo_cell_ids(
            halo_spans,
            cells,
            config,
            region_id=region_id,
        )
        width = int(np.ceil(config.length_m / resolution))
        height = int(np.ceil(config.width_m / resolution))
        zero_row = height // 2
        halo_outline.points = _cell_footprint_outline_points(
            {
                (int(cell_id % width), int(cell_id // width) - zero_row)
                for cell_id in halo_ids
            },
            resolution,
            max(cell.max_z_m for cell in region_cells) + 0.025,
        )
        result.markers.append(halo_outline)
    return result


def build_artifact_threshold_cell_markers(
    header: Header,
    config: FrontCostmapConfig,
    candidate_cell_ids: np.ndarray,
    low_support_cell_ids: np.ndarray,
) -> MarkerArray:
    """Show exact base-frame cells evaluated by the support threshold."""

    geometry = np.asarray(
        [config.length_m, config.width_m, config.resolution_m],
        dtype=np.float64,
    )
    if not np.isfinite(geometry).all() or np.any(geometry <= 0.0):
        raise ValueError("artifact threshold marker grid must be positive")
    width = int(np.ceil(config.length_m / config.resolution_m))
    height = int(np.ceil(config.width_m / config.resolution_m))
    origin_y_m = -(height * config.resolution_m) / 2.0
    result = MarkerArray()
    result.markers.append(
        _cell_list_marker(
            header,
            "artifact_threshold_candidate_cells",
            candidate_cell_ids,
            width,
            height,
            origin_y_m,
            config.resolution_m,
            color=(0.05, 0.75, 1.0, 0.22),
            z_m=0.02,
        )
    )
    result.markers.append(
        _cell_list_marker(
            header,
            "artifact_threshold_low_support_cells",
            low_support_cell_ids,
            width,
            height,
            origin_y_m,
            config.resolution_m,
            color=(1.0, 0.85, 0.0, 0.48),
            z_m=0.035,
        )
    )
    return result


def _cell_list_marker(
    header: Header,
    namespace: str,
    cell_ids: np.ndarray,
    width: int,
    height: int,
    origin_y_m: float,
    resolution_m: float,
    *,
    color: tuple[float, float, float, float],
    z_m: float,
) -> Marker:
    """Build one efficient cube-list marker from flat front-grid IDs."""

    ids = np.asarray(cell_ids, dtype=np.int64)
    if ids.ndim != 1:
        raise ValueError("artifact threshold cell IDs must have shape (N,)")
    if np.any((ids < 0) | (ids >= width * height)):
        raise ValueError(
            "artifact threshold marker cell ID is outside the grid"
        )

    marker = Marker()
    marker.header = header
    marker.ns = namespace
    marker.id = 0
    marker.type = Marker.CUBE_LIST
    marker.action = Marker.ADD if ids.size else Marker.DELETE
    marker.pose.orientation.w = 1.0
    marker.scale.x = resolution_m * 0.96
    marker.scale.y = resolution_m * 0.96
    marker.scale.z = 0.02
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
    marker.frame_locked = True
    rows = ids // width
    cols = ids % width
    for row, col in zip(rows, cols):
        point = Point()
        point.x = (float(col) + 0.5) * resolution_m
        point.y = origin_y_m + (float(row) + 0.5) * resolution_m
        point.z = z_m
        marker.points.append(point)
    return marker


def _cell_box_triangle_points(
    cell: ArtifactGridCell,
    resolution_m: float,
) -> list[Point]:
    """Return twelve triangles for one height-bounded costmap cell."""

    min_x = cell.forward_cell * resolution_m
    max_x = min_x + resolution_m
    min_y = cell.lateral_cell * resolution_m
    max_y = min_y + resolution_m
    corners = [
        (min_x, min_y, cell.min_z_m),
        (max_x, min_y, cell.min_z_m),
        (max_x, max_y, cell.min_z_m),
        (min_x, max_y, cell.min_z_m),
        (min_x, min_y, cell.max_z_m),
        (max_x, min_y, cell.max_z_m),
        (max_x, max_y, cell.max_z_m),
        (min_x, max_y, cell.max_z_m),
    ]
    triangles = (
        (0, 2, 1), (0, 3, 2),
        (4, 5, 6), (4, 6, 7),
        (0, 1, 5), (0, 5, 4),
        (1, 2, 6), (1, 6, 5),
        (2, 3, 7), (2, 7, 6),
        (3, 0, 4), (3, 4, 7),
    )
    points = []
    for triangle in triangles:
        for index in triangle:
            point = Point()
            point.x, point.y, point.z = corners[index]
            points.append(point)
    return points


def _cell_footprint_outline_points(
    cells: set[tuple[int, int]],
    resolution_m: float,
    z_m: float,
) -> list[Point]:
    """Return only the exterior line segments of a staircase footprint."""

    points = []
    for col, row in sorted(cells):
        min_x = col * resolution_m
        max_x = min_x + resolution_m
        min_y = row * resolution_m
        max_y = min_y + resolution_m
        edges = []
        if (col - 1, row) not in cells:
            edges.append(((min_x, min_y), (min_x, max_y)))
        if (col + 1, row) not in cells:
            edges.append(((max_x, min_y), (max_x, max_y)))
        if (col, row - 1) not in cells:
            edges.append(((min_x, min_y), (max_x, min_y)))
        if (col, row + 1) not in cells:
            edges.append(((min_x, max_y), (max_x, max_y)))
        for start, end in edges:
            for x_m, y_m in (start, end):
                point = Point()
                point.x = x_m
                point.y = y_m
                point.z = z_m
                points.append(point)
    return points


__all__ = [
    "build_artifact_grid_markers",
    "build_artifact_threshold_cell_markers",
]
