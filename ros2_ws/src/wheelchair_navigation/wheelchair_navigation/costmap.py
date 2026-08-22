"""Vectorized local obstacle-grid construction for the wheelchair."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class LocalCostmapConfig:
    size_m: float = 8.0
    resolution_m: float = 0.1
    min_height_m: float = 0.05
    max_height_m: float = 1.5
    min_range_m: float = 0.30
    max_range_m: float = 4.0
    occupied_cost: int = 100
    unknown_cost: int = -1


@dataclass(frozen=True)
class FrontCostmapConfig:
    length_m: float = 4.0
    width_m: float = 8.0
    resolution_m: float = 0.1
    fov_deg: float = 180.0
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
    front_points: int = 0
    front_occupied_cells: int = 0


def make_local_costmaps(
    points_base: np.ndarray,
    config: LocalCostmapConfig = LocalCostmapConfig(),
    self_filter_boxes: tuple[SelfFilterBox, ...] = (),
) -> tuple[np.ndarray, CostmapStats]:
    """Return the raw local obstacle grid and filtering statistics."""

    _validate_config(config)
    accepted, counts = filter_obstacle_points(points_base, config, self_filter_boxes)
    raw = make_full_raw_grid(accepted, config)
    stats = make_costmap_stats(counts, accepted, raw, config)
    return raw, stats


def make_local_and_front_costmaps(
    points_base: np.ndarray,
    config: LocalCostmapConfig = LocalCostmapConfig(),
    front_config: FrontCostmapConfig = FrontCostmapConfig(),
    self_filter_boxes: tuple[SelfFilterBox, ...] = (),
) -> tuple[np.ndarray, np.ndarray, CostmapStats]:
    """Build full-surround and robot-forward raw obstacle grids."""

    _validate_config(config)
    _validate_front_config(front_config)
    accepted, counts = filter_obstacle_points(points_base, config, self_filter_boxes)

    raw = make_full_raw_grid(accepted, config)
    front_points = select_front_points(accepted, front_config)
    front = make_front_grid(front_points, front_config)
    stats = make_costmap_stats(
        counts,
        accepted,
        raw,
        config,
        front_points=front_points,
        front=front,
        front_config=front_config,
    )
    return raw, front, stats


def make_full_raw_grid(
    accepted_points: np.ndarray,
    config: LocalCostmapConfig,
) -> np.ndarray:
    """Rasterize filtered points into the base-link-centred raw grid."""

    cell_count = int(np.ceil(config.size_m / config.resolution_m))
    origin_m = grid_origin_m(config)
    return rasterize_points(
        accepted_points,
        origin_x_m=origin_m,
        origin_y_m=origin_m,
        width=cell_count,
        height=cell_count,
        resolution_m=config.resolution_m,
        occupied_cost=config.occupied_cost,
    )


def select_front_points(
    accepted_points: np.ndarray,
    config: FrontCostmapConfig,
) -> np.ndarray:
    """Select the robot-forward sector after points are in ``base_link``."""

    _validate_front_config(config)
    return accepted_points[points_in_front_fov(accepted_points, config.fov_deg)]


def front_point_cell_ids(
    points_base: np.ndarray,
    config: FrontCostmapConfig,
) -> tuple[np.ndarray, np.ndarray, int]:
    """Return front-grid validity, flat cell IDs, and total cell count."""

    _validate_front_config(config)
    points = np.asarray(points_base, dtype=np.float32)
    if points.size == 0:
        points = np.empty((0, 3), dtype=np.float32)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points_base must have shape (N, 3)")

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
    with np.errstate(invalid="ignore"):
        in_fov = points_in_front_fov(points, config.fov_deg)
    valid = (
        finite_xy
        & in_fov
        & (cols >= 0)
        & (cols < width)
        & (rows >= 0)
        & (rows < height)
    )
    cell_ids = rows.astype(np.int64) * width + cols.astype(np.int64)
    return valid, cell_ids, width * height


def make_front_grid(
    front_points: np.ndarray,
    config: FrontCostmapConfig,
) -> np.ndarray:
    """Rasterize selected front points into an X-forward rectangular grid."""

    front_width = int(np.ceil(config.length_m / config.resolution_m))
    front_height = int(np.ceil(config.width_m / config.resolution_m))
    front_origin_y = -(front_height * config.resolution_m) / 2.0
    return rasterize_points(
        front_points,
        origin_x_m=0.0,
        origin_y_m=front_origin_y,
        width=front_width,
        height=front_height,
        resolution_m=config.resolution_m,
        occupied_cost=config.occupied_cost,
    )


def make_costmap_stats(
    counts: dict[str, int],
    accepted_points: np.ndarray,
    raw: np.ndarray | None,
    config: LocalCostmapConfig,
    *,
    front_points: np.ndarray | None = None,
    front: np.ndarray | None = None,
    front_config: FrontCostmapConfig | None = None,
) -> CostmapStats:
    """Build consistent counters for any explicitly selected map outputs."""

    return CostmapStats(
        input_points=counts["input_points"],
        finite_points=counts["finite_points"],
        height_range_points=counts["height_range_points"],
        self_filtered_points=counts["self_filtered_points"],
        accepted_points=int(accepted_points.shape[0]),
        occupied_cells=(
            int(np.count_nonzero(raw == config.occupied_cost))
            if raw is not None
            else 0
        ),
        front_points=(
            int(front_points.shape[0]) if front_points is not None else 0
        ),
        front_occupied_cells=(
            int(np.count_nonzero(front == front_config.occupied_cost))
            if front is not None and front_config is not None
            else 0
        ),
    )


def filter_obstacle_points(
    points_base: np.ndarray,
    config: LocalCostmapConfig,
    self_filter_boxes: tuple[SelfFilterBox, ...] = (),
) -> tuple[np.ndarray, dict[str, int]]:
    """Filter obstacle points once so multiple map layers can reuse them."""

    points = np.asarray(points_base, dtype=np.float32)
    accepted_mask, counts = obstacle_point_mask(
        points, config, self_filter_boxes
    )
    return points[accepted_mask], counts


def obstacle_point_mask(
    points_base: np.ndarray,
    config: LocalCostmapConfig,
    self_filter_boxes: tuple[SelfFilterBox, ...] = (),
) -> tuple[np.ndarray, dict[str, int]]:
    """Return the shared height/range/self acceptance mask and counters."""

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
    return filtered_mask & ~self_mask, {
        "input_points": input_points,
        "finite_points": finite_points,
        "height_range_points": height_range_points,
        "self_filtered_points": self_filtered_points,
    }


def points_in_front_fov(points: np.ndarray, fov_deg: float) -> np.ndarray:
    """Return points inside a base-link-centred forward angular sector."""

    points = np.asarray(points)
    if points.size == 0:
        return np.zeros(points.shape[0], dtype=bool)
    half_fov_rad = np.deg2rad(fov_deg / 2.0)
    angles = np.arctan2(points[:, 1], points[:, 0])
    return np.abs(angles) <= half_fov_rad + 1e-7


def rasterize_points(
    points: np.ndarray,
    *,
    origin_x_m: float,
    origin_y_m: float,
    width: int,
    height: int,
    resolution_m: float,
    occupied_cost: int,
) -> np.ndarray:
    """Rasterize XY points into an occupancy grid with explicit geometry."""

    grid = np.zeros((height, width), dtype=np.int8)
    if not points.size:
        return grid

    cols = np.floor((points[:, 0] - origin_x_m) / resolution_m).astype(np.int32)
    rows = np.floor((points[:, 1] - origin_y_m) / resolution_m).astype(np.int32)
    in_grid = (
        (cols >= 0)
        & (cols < width)
        & (rows >= 0)
        & (rows < height)
    )
    grid[rows[in_grid], cols[in_grid]] = occupied_cost
    return grid


def points_in_box(points: np.ndarray, box: SelfFilterBox) -> np.ndarray:
    """Return a vectorized mask for points inside a closed 3D box."""

    with np.errstate(invalid="ignore"):
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
            float(value) for value in values[start:start + 6]
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


def _validate_front_config(config: FrontCostmapConfig) -> None:
    if config.length_m <= 0.0 or config.width_m <= 0.0:
        raise ValueError("front map dimensions must be positive")
    if config.resolution_m <= 0.0:
        raise ValueError("front map resolution must be positive")
    if config.fov_deg <= 0.0 or config.fov_deg > 180.0:
        raise ValueError("front FOV must be in (0, 180] degrees")


def validate_mapping_configs(
    config: LocalCostmapConfig,
    front_config: FrontCostmapConfig,
) -> None:
    """Validate both map configurations before a selected-output build."""

    _validate_config(config)
    _validate_front_config(front_config)


__all__ = [
    "CostmapStats",
    "FrontCostmapConfig",
    "LocalCostmapConfig",
    "SelfFilterBox",
    "filter_obstacle_points",
    "front_point_cell_ids",
    "grid_origin_m",
    "make_costmap_stats",
    "make_front_grid",
    "make_full_raw_grid",
    "make_local_and_front_costmaps",
    "make_local_costmaps",
    "obstacle_point_mask",
    "parse_self_filter_boxes",
    "points_in_front_fov",
    "points_in_box",
    "rasterize_points",
    "select_front_points",
    "validate_mapping_configs",
    "world_to_cell",
]
