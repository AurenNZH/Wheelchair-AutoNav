"""Geometry shared by the L2 support filter and Nav2 costmap input."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class LocalCostmapConfig:
    size_m: float = 8.0
    resolution_m: float = 0.1
    min_height_m: float = 0.05
    max_height_m: float = 1.5
    min_range_m: float = 0.45
    max_range_m: float = 4.0


@dataclass(frozen=True)
class FrontCostmapConfig:
    length_m: float = 4.0
    width_m: float = 8.0
    resolution_m: float = 0.1
    fov_deg: float = 180.0


def obstacle_point_mask(
    points_base: np.ndarray,
    config: LocalCostmapConfig,
) -> tuple[np.ndarray, dict[str, int]]:
    """Return Nav2's height/range eligibility mask and useful counters."""

    _validate_config(config)
    points = _points_array(points_base)
    finite = np.isfinite(points).all(axis=1)
    ranges = np.linalg.norm(points[:, :2], axis=1)
    with np.errstate(invalid="ignore"):
        accepted = (
            finite
            & (points[:, 2] >= config.min_height_m)
            & (points[:, 2] <= config.max_height_m)
            & (ranges >= config.min_range_m)
            & (ranges <= config.max_range_m)
        )
    return accepted, {
        "input_points": int(points.shape[0]),
        "finite_points": int(np.count_nonzero(finite)),
        "height_range_points": int(np.count_nonzero(accepted)),
    }


def minimum_range_rejection_mask(
    points_base: np.ndarray,
    min_range_m: float,
) -> np.ndarray:
    """Return finite points strictly inside a horizontal minimum range.

    A point exactly on the configured boundary remains eligible.
    """

    minimum = float(min_range_m)
    if not np.isfinite(minimum) or minimum < 0.0:
        raise ValueError("minimum range must be finite and non-negative")
    points = _points_array(points_base)
    finite = np.isfinite(points).all(axis=1)
    ranges = np.linalg.norm(points[:, :2], axis=1)
    with np.errstate(invalid="ignore"):
        return finite & (ranges < minimum)


def front_point_cell_ids(
    points_base: np.ndarray,
    config: FrontCostmapConfig,
) -> tuple[np.ndarray, np.ndarray, int]:
    """Return front-grid validity, flat cell IDs, and total cell count."""

    _validate_front_config(config)
    points = _points_array(points_base)
    width = int(np.ceil(config.length_m / config.resolution_m))
    height = int(np.ceil(config.width_m / config.resolution_m))
    origin_y_m = -(height * config.resolution_m) / 2.0
    finite_xy = np.isfinite(points[:, :2]).all(axis=1)
    cols = np.full(points.shape[0], -1, dtype=np.int32)
    rows = np.full(points.shape[0], -1, dtype=np.int32)
    finite_indices = np.flatnonzero(finite_xy)
    cols[finite_indices] = np.floor(
        points[finite_indices, 0] / config.resolution_m
    ).astype(np.int32)
    rows[finite_indices] = np.floor(
        (points[finite_indices, 1] - origin_y_m) / config.resolution_m
    ).astype(np.int32)
    valid = (
        finite_xy
        & points_in_front_fov(points, config.fov_deg)
        & (cols >= 0)
        & (cols < width)
        & (rows >= 0)
        & (rows < height)
    )
    return valid, rows.astype(np.int64) * width + cols.astype(np.int64), width * height


def points_in_front_fov(points: np.ndarray, fov_deg: float) -> np.ndarray:
    """Return points inside a base-link-centred forward angular sector."""

    points = _points_array(points)
    if not points.size:
        return np.zeros(points.shape[0], dtype=bool)
    half_fov_rad = np.deg2rad(fov_deg / 2.0)
    angles = np.arctan2(points[:, 1], points[:, 0])
    with np.errstate(invalid="ignore"):
        return np.abs(angles) <= half_fov_rad + 1e-7


def validate_mapping_configs(
    config: LocalCostmapConfig,
    front_config: FrontCostmapConfig,
) -> None:
    _validate_config(config)
    _validate_front_config(front_config)


def _points_array(points: np.ndarray) -> np.ndarray:
    array = np.asarray(points, dtype=np.float32)
    if array.size == 0:
        array = np.empty((0, 3), dtype=np.float32)
    if array.ndim != 2 or array.shape[1] != 3:
        raise ValueError("points_base must have shape (N, 3)")
    return array


def _validate_config(config: LocalCostmapConfig) -> None:
    geometry = np.asarray(
        [
            config.size_m,
            config.resolution_m,
            config.min_height_m,
            config.max_height_m,
            config.min_range_m,
            config.max_range_m,
        ],
        dtype=np.float64,
    )
    if not np.isfinite(geometry).all():
        raise ValueError("costmap geometry must be finite")
    if config.size_m <= 0.0 or config.resolution_m <= 0.0:
        raise ValueError("map size and resolution must be positive")
    if config.min_height_m > config.max_height_m:
        raise ValueError("minimum height exceeds maximum height")
    if config.min_range_m < 0.0 or config.min_range_m > config.max_range_m:
        raise ValueError("invalid range limits")


def _validate_front_config(config: FrontCostmapConfig) -> None:
    geometry = np.asarray(
        [config.length_m, config.width_m, config.resolution_m, config.fov_deg],
        dtype=np.float64,
    )
    if not np.isfinite(geometry).all():
        raise ValueError("front-grid geometry must be finite")
    if config.length_m <= 0.0 or config.width_m <= 0.0:
        raise ValueError("front map dimensions must be positive")
    if config.resolution_m <= 0.0:
        raise ValueError("front map resolution must be positive")
    if config.fov_deg <= 0.0 or config.fov_deg > 180.0:
        raise ValueError("front FOV must be in (0, 180] degrees")


__all__ = [
    "FrontCostmapConfig",
    "LocalCostmapConfig",
    "front_point_cell_ids",
    "minimum_range_rejection_mask",
    "obstacle_point_mask",
    "points_in_front_fov",
    "validate_mapping_configs",
]
