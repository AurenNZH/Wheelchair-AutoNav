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
class SupportGridConfig:
    """Robot-relative grid used for point-support counting."""

    origin_x_m: float = -0.6
    origin_y_m: float = -4.0
    width_m: float = 5.0
    height_m: float = 8.0
    resolution_m: float = 0.1


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


def point_cell_ids(
    points_base: np.ndarray,
    config: SupportGridConfig,
) -> tuple[np.ndarray, np.ndarray, int]:
    """Return merged-grid validity, flat cell IDs, and total cell count."""

    _validate_support_grid_config(config)
    points = _points_array(points_base)
    width = int(np.ceil(config.width_m / config.resolution_m))
    height = int(np.ceil(config.height_m / config.resolution_m))
    finite_xy = np.isfinite(points[:, :2]).all(axis=1)
    cols = np.full(points.shape[0], -1, dtype=np.int32)
    rows = np.full(points.shape[0], -1, dtype=np.int32)
    finite_indices = np.flatnonzero(finite_xy)
    cols[finite_indices] = np.floor(
        (points[finite_indices, 0] - config.origin_x_m)
        / config.resolution_m
    ).astype(np.int32)
    rows[finite_indices] = np.floor(
        (points[finite_indices, 1] - config.origin_y_m)
        / config.resolution_m
    ).astype(np.int32)
    valid = (
        finite_xy
        & (cols >= 0)
        & (cols < width)
        & (rows >= 0)
        & (rows < height)
    )
    return valid, rows.astype(np.int64) * width + cols.astype(np.int64), width * height


def validate_mapping_configs(
    config: LocalCostmapConfig,
    support_grid_config: SupportGridConfig,
) -> None:
    _validate_config(config)
    _validate_support_grid_config(support_grid_config)


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


def _validate_support_grid_config(config: SupportGridConfig) -> None:
    geometry = np.asarray(
        [
            config.origin_x_m,
            config.origin_y_m,
            config.width_m,
            config.height_m,
            config.resolution_m,
        ],
        dtype=np.float64,
    )
    if not np.isfinite(geometry).all():
        raise ValueError("support-grid geometry must be finite")
    if config.width_m <= 0.0 or config.height_m <= 0.0:
        raise ValueError("support-grid dimensions must be positive")
    if config.resolution_m <= 0.0:
        raise ValueError("support-grid resolution must be positive")


__all__ = [
    "LocalCostmapConfig",
    "SupportGridConfig",
    "minimum_range_rejection_mask",
    "obstacle_point_mask",
    "point_cell_ids",
    "validate_mapping_configs",
]
