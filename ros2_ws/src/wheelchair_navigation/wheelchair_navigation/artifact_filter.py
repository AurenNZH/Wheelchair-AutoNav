"""Pure geometry for one calibrated chassis-artifact rule per lidar."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from wheelchair_navigation.costmap import FrontCostmapConfig


@dataclass(frozen=True)
class ArtifactBox:
    """Inclusive axis-aligned hard-removal box in the target frame."""

    min_x_m: float
    max_x_m: float
    min_y_m: float
    max_y_m: float
    min_z_m: float
    max_z_m: float


def parse_artifact_box(values) -> ArtifactBox:
    """Parse and validate six XYZ bounds."""

    if values is None or len(values) != 6:
        raise ValueError("artifact_box must contain six XYZ bounds")
    bounds = np.asarray(values, dtype=np.float64)
    if not np.isfinite(bounds).all():
        raise ValueError("artifact box bounds must be finite")
    box = ArtifactBox(*[float(value) for value in bounds])
    if (
        box.min_x_m >= box.max_x_m
        or box.min_y_m >= box.max_y_m
        or box.min_z_m >= box.max_z_m
    ):
        raise ValueError("artifact box minimums must be below maximums")
    return box


def points_in_artifact_box(
    points_base: np.ndarray, box: ArtifactBox
) -> np.ndarray:
    """Return inclusive hard-removal membership for finite XYZ points."""

    points = _points_array(points_base)
    finite = np.isfinite(points).all(axis=1)
    with np.errstate(invalid="ignore"):
        return (
            finite
            & (points[:, 0] >= box.min_x_m)
            & (points[:, 0] <= box.max_x_m)
            & (points[:, 1] >= box.min_y_m)
            & (points[:, 1] <= box.max_y_m)
            & (points[:, 2] >= box.min_z_m)
            & (points[:, 2] <= box.max_z_m)
        )


def artifact_halo_cell_ids(
    box: ArtifactBox,
    config: FrontCostmapConfig,
    margin_m: float,
) -> np.ndarray:
    """Return front-grid cells intersecting the box's expanded XY footprint."""

    margin = float(margin_m)
    if not np.isfinite(margin) or margin < 0.0:
        raise ValueError("artifact halo margin must be finite and non-negative")
    resolution = float(config.resolution_m)
    width = int(np.ceil(config.length_m / resolution))
    height = int(np.ceil(config.width_m / resolution))
    if width < 1 or height < 1 or not np.isfinite(resolution) or resolution <= 0.0:
        raise ValueError("artifact halo grid must be positive")
    origin_y = -(height * resolution) / 2.0
    min_col = int(np.floor((box.min_x_m - margin) / resolution))
    max_col = int(np.floor((box.max_x_m + margin) / resolution))
    min_row = int(np.floor((box.min_y_m - margin - origin_y) / resolution))
    max_row = int(np.floor((box.max_y_m + margin - origin_y) / resolution))
    if max_col < 0 or min_col >= width or max_row < 0 or min_row >= height:
        return np.empty(0, dtype=np.int64)
    min_col = max(0, min(width - 1, min_col))
    max_col = max(0, min(width - 1, max_col))
    min_row = max(0, min(height - 1, min_row))
    max_row = max(0, min(height - 1, max_row))
    if min_col > max_col or min_row > max_row:
        return np.empty(0, dtype=np.int64)
    return np.asarray(
        [
            row * width + col
            for row in range(min_row, max_row + 1)
            for col in range(min_col, max_col + 1)
        ],
        dtype=np.int64,
    )


def _points_array(points: np.ndarray) -> np.ndarray:
    array = np.asarray(points, dtype=np.float32)
    if array.size == 0:
        array = np.empty((0, 3), dtype=np.float32)
    if array.ndim != 2 or array.shape[1] != 3:
        raise ValueError("points_base must have shape (N, 3)")
    return array


__all__ = [
    "ArtifactBox",
    "artifact_halo_cell_ids",
    "parse_artifact_box",
    "points_in_artifact_box",
]
