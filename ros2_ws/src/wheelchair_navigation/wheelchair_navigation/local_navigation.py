"""Vectorized local obstacle-grid construction for the wheelchair."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass

import numpy as np
from scipy.ndimage import binary_dilation, label


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
class FrontCostmapConfig:
    length_m: float = 4.0
    width_m: float = 8.0
    resolution_m: float = 0.1
    fov_deg: float = 180.0
    inflation_radius_m: float = 0.0
    occupied_cost: int = 100
    unknown_cost: int = -1


@dataclass(frozen=True)
class GhostFilterConfig:
    """Conservative shadow-filter settings for a 2D obstacle grid."""

    min_component_cells: int = 2
    history_frames: int = 3
    min_hits: int = 2
    match_radius_cells: int = 1
    reset_gap_s: float = 0.5
    occupied_cost: int = 100


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


@dataclass(frozen=True)
class GhostFilterStats:
    raw_cells: int
    component_count: int
    strong_component_cells: int
    isolated_cells: int
    temporal_rescued_cells: int
    filtered_cells: int
    rejected_cells: int
    history_reset: bool


class TemporalGhostFilter:
    """Reject only isolated, non-persistent cells from a shadow map."""

    def __init__(self, config: GhostFilterConfig = GhostFilterConfig()) -> None:
        validate_ghost_filter_config(config)
        self.config = config
        self._history = deque(maxlen=max(0, config.history_frames - 1))
        self._last_stamp_ns: int | None = None

    def reset(self) -> None:
        self._history.clear()
        self._last_stamp_ns = None

    def filter(
        self, raw_grid: np.ndarray, stamp_ns: int
    ) -> tuple[np.ndarray, np.ndarray, GhostFilterStats]:
        """Return filtered/rejected grids without modifying ``raw_grid``."""

        grid = np.asarray(raw_grid)
        if grid.ndim != 2:
            raise ValueError("ghost filter input must be a 2D grid")
        if stamp_ns <= 0:
            raise ValueError("ghost filter timestamp must be positive")

        history_reset = False
        if self._last_stamp_ns is not None:
            gap_ns = stamp_ns - self._last_stamp_ns
            if gap_ns <= 0 or gap_ns > int(self.config.reset_gap_s * 1e9):
                self._history.clear()
                history_reset = True
        self._last_stamp_ns = stamp_ns

        occupied = grid >= self.config.occupied_cost
        connectivity = np.ones((3, 3), dtype=np.uint8)
        labels, component_count = label(occupied, structure=connectivity)
        component_sizes = np.bincount(labels.reshape(-1))
        strong = occupied & (
            component_sizes[labels] >= self.config.min_component_cells
        )
        isolated = occupied & ~strong

        hit_count = occupied.astype(np.uint8)
        radius = self.config.match_radius_cells
        match_structure = np.ones(
            (2 * radius + 1, 2 * radius + 1), dtype=bool
        )
        for previous in self._history:
            matched = (
                previous
                if radius == 0
                else binary_dilation(previous, structure=match_structure)
            )
            hit_count += matched.astype(np.uint8)
        temporal = isolated & (hit_count >= self.config.min_hits)
        filtered_mask = strong | temporal
        rejected_mask = occupied & ~filtered_mask

        filtered = np.zeros_like(grid)
        rejected = np.zeros_like(grid)
        filtered[filtered_mask] = self.config.occupied_cost
        rejected[rejected_mask] = self.config.occupied_cost
        self._history.append(np.array(occupied, copy=True))

        stats = GhostFilterStats(
            raw_cells=int(np.count_nonzero(occupied)),
            component_count=int(component_count),
            strong_component_cells=int(np.count_nonzero(strong)),
            isolated_cells=int(np.count_nonzero(isolated)),
            temporal_rescued_cells=int(np.count_nonzero(temporal)),
            filtered_cells=int(np.count_nonzero(filtered_mask)),
            rejected_cells=int(np.count_nonzero(rejected_mask)),
            history_reset=history_reset,
        )
        return filtered, rejected, stats


def make_local_costmaps(
    points_base: np.ndarray,
    config: LocalCostmapConfig = LocalCostmapConfig(),
    self_filter_boxes: tuple[SelfFilterBox, ...] = (),
) -> tuple[np.ndarray, np.ndarray, CostmapStats]:
    """Return raw obstacles, an optionally inflated layer, and filter stats."""

    _validate_config(config)
    accepted, counts = filter_obstacle_points(points_base, config, self_filter_boxes)
    raw = make_full_raw_grid(accepted, config)
    inflated = inflate_grid(
        raw,
        config.inflation_radius_m,
        config.resolution_m,
        config.occupied_cost,
    )
    stats = make_costmap_stats(counts, accepted, raw, config)
    return raw, inflated, stats


def make_local_and_front_costmaps(
    points_base: np.ndarray,
    config: LocalCostmapConfig = LocalCostmapConfig(),
    front_config: FrontCostmapConfig = FrontCostmapConfig(),
    self_filter_boxes: tuple[SelfFilterBox, ...] = (),
) -> tuple[np.ndarray, np.ndarray, np.ndarray, CostmapStats]:
    """Build full-surround raw/derived grids and a forward-only grid."""

    _validate_config(config)
    _validate_front_config(front_config)
    accepted, counts = filter_obstacle_points(points_base, config, self_filter_boxes)

    raw = make_full_raw_grid(accepted, config)
    inflated = inflate_grid(
        raw,
        config.inflation_radius_m,
        config.resolution_m,
        config.occupied_cost,
    )

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
    return raw, inflated, front, stats


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


def make_front_grid(
    front_points: np.ndarray,
    config: FrontCostmapConfig,
) -> np.ndarray:
    """Rasterize selected front points into an X-forward rectangular grid."""

    front = make_front_raw_grid(front_points, config)
    return inflate_grid(
        front,
        config.inflation_radius_m,
        config.resolution_m,
        config.occupied_cost,
    )


def make_front_raw_grid(
    front_points: np.ndarray,
    config: FrontCostmapConfig,
) -> np.ndarray:
    """Rasterize front points without inflating isolated evidence."""

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
    return accepted, {
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


def inflate_obstacles(grid: np.ndarray, config: LocalCostmapConfig) -> np.ndarray:
    """Inflate occupied cells once, leaving the raw grid unchanged."""

    return inflate_grid(
        grid,
        config.inflation_radius_m,
        config.resolution_m,
        config.occupied_cost,
    )


def inflate_grid(
    grid: np.ndarray,
    inflation_radius_m: float,
    resolution_m: float,
    occupied_cost: int,
) -> np.ndarray:
    """Inflate an occupancy grid using a circular metric footprint."""

    inflated = np.array(grid, copy=True)
    if inflation_radius_m <= 0.0:
        return inflated

    radius_cells = int(np.ceil(inflation_radius_m / resolution_m))
    offsets = np.arange(-radius_cells, radius_cells + 1, dtype=np.float32)
    yy, xx = np.meshgrid(offsets, offsets, indexing="ij")
    footprint = (
        np.hypot(xx, yy) * resolution_m
        <= inflation_radius_m + 1e-6
    )
    occupied = grid >= occupied_cost
    inflated[binary_dilation(occupied, structure=footprint)] = occupied_cost
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


def _validate_front_config(config: FrontCostmapConfig) -> None:
    if config.length_m <= 0.0 or config.width_m <= 0.0:
        raise ValueError("front map dimensions must be positive")
    if config.resolution_m <= 0.0:
        raise ValueError("front map resolution must be positive")
    if config.fov_deg <= 0.0 or config.fov_deg > 180.0:
        raise ValueError("front FOV must be in (0, 180] degrees")
    if config.inflation_radius_m < 0.0:
        raise ValueError("front inflation radius must be non-negative")


def validate_mapping_configs(
    config: LocalCostmapConfig,
    front_config: FrontCostmapConfig,
) -> None:
    """Validate both map configurations before a selected-output build."""

    _validate_config(config)
    _validate_front_config(front_config)


def validate_ghost_filter_config(config: GhostFilterConfig) -> None:
    """Reject settings that cannot produce a bounded temporal filter."""

    if config.min_component_cells < 1:
        raise ValueError("ghost-filter component size must be positive")
    if config.history_frames < 1:
        raise ValueError("ghost-filter history length must be positive")
    if config.min_hits < 1 or config.min_hits > config.history_frames:
        raise ValueError("ghost-filter minimum hits must be within history")
    if config.match_radius_cells < 0:
        raise ValueError("ghost-filter match radius must be non-negative")
    if config.reset_gap_s <= 0.0:
        raise ValueError("ghost-filter reset gap must be positive")
    if config.occupied_cost <= 0:
        raise ValueError("ghost-filter occupied cost must be positive")


__all__ = [
    "CostmapStats",
    "FrontCostmapConfig",
    "GhostFilterConfig",
    "GhostFilterStats",
    "LocalCostmapConfig",
    "SelfFilterBox",
    "TemporalGhostFilter",
    "filter_obstacle_points",
    "grid_origin_m",
    "inflate_grid",
    "inflate_obstacles",
    "make_costmap_stats",
    "make_front_grid",
    "make_front_raw_grid",
    "make_full_raw_grid",
    "make_local_and_front_costmaps",
    "make_local_costmaps",
    "parse_self_filter_boxes",
    "points_in_front_fov",
    "points_in_box",
    "rasterize_points",
    "select_front_points",
    "validate_ghost_filter_config",
    "validate_mapping_configs",
    "world_to_cell",
]
