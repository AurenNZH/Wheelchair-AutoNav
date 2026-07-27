"""Vectorized local obstacle-grid construction for the wheelchair."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.ndimage import binary_dilation


@dataclass(frozen=True)
class LocalCostmapConfig:
    size_m: float = 8.0
    resolution_m: float = 0.1
    min_height_m: float = 0.05
    max_height_m: float = 1.5
    min_range_m: float = 0.30
    max_range_m: float = 4.0
    inflation_radius_m: float = 0.0
    occupied_cost: int = 100
    unknown_cost: int = -1


@dataclass(frozen=True)
class SelfFilterBox:
    """Axis-aligned exclusion box expressed in ``base_link``."""

    min_x_m: float
    max_x_m: float
    min_y_m: float
    max_y_m: float
    min_z_m: float
    max_z_m: float


@dataclass(frozen=True)
class CostmapStats:
    input_points: int
    finite_points: int
    height_range_points: int
    self_filtered_points: int
    accepted_points: int
    occupied_cells: int


def make_local_costmaps(
    points_base: np.ndarray,
    config: LocalCostmapConfig = LocalCostmapConfig(),
    self_filter_boxes: tuple[SelfFilterBox, ...] = (),
) -> tuple[np.ndarray, np.ndarray, CostmapStats]:
    """Return raw obstacles, an optionally inflated layer, and filter stats."""

    _validate_config(config)
    cell_count = int(np.ceil(config.size_m / config.resolution_m))
    raw = np.zeros((cell_count, cell_count), dtype=np.int8)

    points = np.asarray(points_base, dtype=np.float32)
    if points.size == 0:
        points = np.empty((0, 3), dtype=np.float32)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points_base must have shape (N, 3)")

    input_points = int(points.shape[0])
    finite_mask = np.isfinite(points).all(axis=1)
    finite_points = int(np.count_nonzero(finite_mask))

    ranges = np.linalg.norm(points[:, :2], axis=1)
    with np.errstate(invalid="ignore"):
        filtered_mask = (
            finite_mask
            & (points[:, 2] >= config.min_height_m)
            & (points[:, 2] <= config.max_height_m)
            & (ranges >= config.min_range_m)
            & (ranges <= config.max_range_m)
        )
    height_range_points = int(np.count_nonzero(filtered_mask))

    self_mask = np.zeros(input_points, dtype=bool)
    for box in self_filter_boxes:
        self_mask |= points_in_box(points, box)
    self_filtered_points = int(np.count_nonzero(filtered_mask & self_mask))
    accepted = points[filtered_mask & ~self_mask]

    if accepted.size:
        origin_m = grid_origin_m(config)
        cols = np.floor((accepted[:, 0] - origin_m) / config.resolution_m).astype(
            np.int32
        )
        rows = np.floor((accepted[:, 1] - origin_m) / config.resolution_m).astype(
            np.int32
        )
        in_grid = (
            (cols >= 0)
            & (cols < cell_count)
            & (rows >= 0)
            & (rows < cell_count)
        )
        raw[rows[in_grid], cols[in_grid]] = config.occupied_cost

    inflated = inflate_obstacles(raw, config)
    stats = CostmapStats(
        input_points=input_points,
        finite_points=finite_points,
        height_range_points=height_range_points,
        self_filtered_points=self_filtered_points,
        accepted_points=int(accepted.shape[0]),
        occupied_cells=int(np.count_nonzero(raw == config.occupied_cost)),
    )
    return raw, inflated, stats


def points_in_box(points: np.ndarray, box: SelfFilterBox) -> np.ndarray:
    """Return a vectorized mask for points inside a closed 3D box."""

    return (
        (points[:, 0] >= box.min_x_m)
        & (points[:, 0] <= box.max_x_m)
        & (points[:, 1] >= box.min_y_m)
        & (points[:, 1] <= box.max_y_m)
        & (points[:, 2] >= box.min_z_m)
        & (points[:, 2] <= box.max_z_m)
    )


def parse_self_filter_boxes(
    values: list[float] | tuple[float, ...] | None, padding_m: float = 0.0
) -> tuple[SelfFilterBox, ...]:
    """Parse flat groups of six base-frame box bounds."""

    if padding_m < 0.0:
        raise ValueError("self-filter padding must be non-negative")
    if values is None:
        return ()
    if len(values) % 6:
        raise ValueError("self_filter_boxes must contain groups of six values")

    boxes = []
    for start in range(0, len(values), 6):
        min_x, max_x, min_y, max_y, min_z, max_z = (
            float(value) for value in values[start : start + 6]
        )
        if min_x > max_x or min_y > max_y or min_z > max_z:
            raise ValueError("self-filter box minimum exceeds maximum")
        boxes.append(
            SelfFilterBox(
                min_x - padding_m,
                max_x + padding_m,
                min_y - padding_m,
                max_y + padding_m,
                min_z - padding_m,
                max_z + padding_m,
            )
        )
    return tuple(boxes)


def inflate_obstacles(grid: np.ndarray, config: LocalCostmapConfig) -> np.ndarray:
    """Inflate occupied cells once, leaving the raw grid unchanged."""

    inflated = np.array(grid, copy=True)
    if config.inflation_radius_m <= 0.0:
        return inflated

    radius_cells = int(np.ceil(config.inflation_radius_m / config.resolution_m))
    offsets = np.arange(-radius_cells, radius_cells + 1, dtype=np.float32)
    yy, xx = np.meshgrid(offsets, offsets, indexing="ij")
    footprint = (
        np.hypot(xx, yy) * config.resolution_m
        <= config.inflation_radius_m + 1e-6
    )
    occupied = grid >= config.occupied_cost
    inflated[binary_dilation(occupied, structure=footprint)] = config.occupied_cost
    return inflated


def grid_origin_m(config: LocalCostmapConfig) -> float:
    cell_count = int(np.ceil(config.size_m / config.resolution_m))
    return -(cell_count * config.resolution_m) / 2.0


def world_to_cell(
    x_m: float,
    y_m: float,
    config: LocalCostmapConfig,
    cell_count: int | None = None,
) -> tuple[int, int] | None:
    """Convert a base-frame point to a local-grid column and row."""

    count = cell_count or int(np.ceil(config.size_m / config.resolution_m))
    origin_m = -(count * config.resolution_m) / 2.0
    col = int(np.floor((x_m - origin_m) / config.resolution_m))
    row = int(np.floor((y_m - origin_m) / config.resolution_m))
    if 0 <= col < count and 0 <= row < count:
        return col, row
    return None


def _validate_config(config: LocalCostmapConfig) -> None:
    if config.size_m <= 0.0 or config.resolution_m <= 0.0:
        raise ValueError("map size and resolution must be positive")
    if config.min_height_m > config.max_height_m:
        raise ValueError("minimum height exceeds maximum height")
    if config.min_range_m < 0.0 or config.min_range_m > config.max_range_m:
        raise ValueError("invalid range limits")
    if config.inflation_radius_m < 0.0:
        raise ValueError("inflation radius must be non-negative")


__all__ = [
    "CostmapStats",
    "LocalCostmapConfig",
    "SelfFilterBox",
    "grid_origin_m",
    "inflate_obstacles",
    "make_local_costmaps",
    "parse_self_filter_boxes",
    "points_in_box",
    "world_to_cell",
]
