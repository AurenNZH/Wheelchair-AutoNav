"""Local costmap and conservative command-proposal algorithms."""

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
    inflation_radius_m: float = 0.45
    occupied_cost: int = 100
    unknown_cost: int = -1


@dataclass(frozen=True)
class TrajectoryConfig:
    horizon_s: float = 2.0
    dt_s: float = 0.2
    linear_speed_mps: float = 0.25
    angular_samples_radps: tuple[float, ...] = (-0.7, -0.35, 0.0, 0.35, 0.7)
    footprint_radius_m: float = 0.45
    goal_heading_rad: float = 0.0
    heading_weight: float = 1.0
    clearance_weight: float = 0.25


@dataclass(frozen=True)
class CommandProposal:
    linear_x_mps: float
    angular_z_radps: float
    safe: bool
    reason: str


@dataclass(frozen=True)
class TrajectoryCandidate:
    angular_z_radps: float
    points_xy: np.ndarray
    safe: bool
    score: float
    min_clearance_m: float


@dataclass(frozen=True)
class SocialZone:
    x_m: float
    y_m: float
    radius_m: float
    cost: int = 80


def make_local_costmap(
    points_base: np.ndarray,
    config: LocalCostmapConfig = LocalCostmapConfig(),
    social_zones: tuple[SocialZone, ...] = (),
) -> np.ndarray:
    """Build a local occupancy costmap centered on the robot base frame."""

    cell_count = int(np.ceil(config.size_m / config.resolution_m))
    if cell_count % 2 == 0:
        cell_count += 1
    grid = np.zeros((cell_count, cell_count), dtype=np.int8)

    if points_base.size:
        points = np.asarray(points_base, dtype=np.float32)
        ranges = np.linalg.norm(points[:, :2], axis=1)
        mask = (
            np.isfinite(points).all(axis=1)
            & (points[:, 2] >= config.min_height_m)
            & (points[:, 2] <= config.max_height_m)
            & (ranges >= config.min_range_m)
            & (ranges <= config.max_range_m)
        )
        for x_m, y_m, _ in points[mask]:
            cell = world_to_cell(float(x_m), float(y_m), config, cell_count)
            if cell is not None:
                grid[cell[1], cell[0]] = config.occupied_cost

    inflate_grid(grid, config.inflation_radius_m, config.resolution_m, config.occupied_cost)
    apply_social_zones(grid, social_zones, config, cell_count)
    return grid


def choose_command(
    costmap: np.ndarray,
    map_config: LocalCostmapConfig = LocalCostmapConfig(),
    trajectory_config: TrajectoryConfig = TrajectoryConfig(),
) -> tuple[CommandProposal, list[TrajectoryCandidate]]:
    candidates = sample_trajectories(costmap, map_config, trajectory_config)
    safe_candidates = [candidate for candidate in candidates if candidate.safe]
    if not safe_candidates:
        return CommandProposal(0.0, 0.0, False, "no_collision_free_trajectory"), candidates

    best = max(safe_candidates, key=lambda candidate: candidate.score)
    return (
        CommandProposal(
            trajectory_config.linear_speed_mps,
            best.angular_z_radps,
            True,
            "collision_free_trajectory",
        ),
        candidates,
    )


def sample_trajectories(
    costmap: np.ndarray,
    map_config: LocalCostmapConfig,
    trajectory_config: TrajectoryConfig,
) -> list[TrajectoryCandidate]:
    candidates: list[TrajectoryCandidate] = []
    for angular_z in trajectory_config.angular_samples_radps:
        points_xy = rollout_arc(
            trajectory_config.linear_speed_mps,
            angular_z,
            trajectory_config.horizon_s,
            trajectory_config.dt_s,
        )
        safe, min_clearance = trajectory_is_safe(
            points_xy,
            costmap,
            map_config,
            trajectory_config.footprint_radius_m,
        )
        heading_error = abs(angular_z - trajectory_config.goal_heading_rad)
        score = (
            -trajectory_config.heading_weight * heading_error
            + trajectory_config.clearance_weight * min_clearance
        )
        candidates.append(TrajectoryCandidate(angular_z, points_xy, safe, score, min_clearance))
    return candidates


def rollout_arc(
    linear_x_mps: float,
    angular_z_radps: float,
    horizon_s: float,
    dt_s: float,
) -> np.ndarray:
    steps = max(1, int(np.ceil(horizon_s / dt_s)))
    x_m = 0.0
    y_m = 0.0
    yaw_rad = 0.0
    points = []
    for _ in range(steps):
        x_m += linear_x_mps * np.cos(yaw_rad) * dt_s
        y_m += linear_x_mps * np.sin(yaw_rad) * dt_s
        yaw_rad += angular_z_radps * dt_s
        points.append((x_m, y_m))
    return np.asarray(points, dtype=np.float32)


def trajectory_is_safe(
    points_xy: np.ndarray,
    costmap: np.ndarray,
    config: LocalCostmapConfig,
    footprint_radius_m: float,
) -> tuple[bool, float]:
    inflated = np.array(costmap, copy=True)
    inflate_grid(inflated, footprint_radius_m, config.resolution_m, config.occupied_cost)
    cell_count = costmap.shape[0]
    min_clearance = config.size_m
    occupied_cells = np.argwhere(costmap >= config.occupied_cost)

    for x_m, y_m in points_xy:
        cell = world_to_cell(float(x_m), float(y_m), config, cell_count)
        if cell is None:
            return False, 0.0
        if inflated[cell[1], cell[0]] >= config.occupied_cost:
            return False, 0.0
        if occupied_cells.size:
            cell_xy = np.array([cell[1], cell[0]], dtype=np.float32)
            distances = np.linalg.norm((occupied_cells - cell_xy) * config.resolution_m, axis=1)
            min_clearance = min(min_clearance, float(np.min(distances)))

    return True, min_clearance


def world_to_cell(
    x_m: float,
    y_m: float,
    config: LocalCostmapConfig,
    cell_count: int,
) -> tuple[int, int] | None:
    origin_m = -config.size_m / 2.0
    col = int(np.floor((x_m - origin_m) / config.resolution_m))
    row = int(np.floor((y_m - origin_m) / config.resolution_m))
    if 0 <= col < cell_count and 0 <= row < cell_count:
        return col, row
    return None


def inflate_grid(grid: np.ndarray, radius_m: float, resolution_m: float, cost: int) -> None:
    radius_cells = int(np.ceil(radius_m / resolution_m))
    if radius_cells <= 0:
        return

    occupied = np.argwhere(grid >= cost)
    height, width = grid.shape
    for row, col in occupied:
        row_min = max(0, row - radius_cells)
        row_max = min(height, row + radius_cells + 1)
        col_min = max(0, col - radius_cells)
        col_max = min(width, col + radius_cells + 1)
        for out_row in range(row_min, row_max):
            for out_col in range(col_min, col_max):
                if (out_row - row) ** 2 + (out_col - col) ** 2 <= radius_cells**2:
                    grid[out_row, out_col] = max(grid[out_row, out_col], cost)


def apply_social_zones(
    grid: np.ndarray,
    zones: tuple[SocialZone, ...],
    config: LocalCostmapConfig,
    cell_count: int,
) -> None:
    for zone in zones:
        center = world_to_cell(zone.x_m, zone.y_m, config, cell_count)
        if center is None:
            continue
        radius_cells = int(np.ceil(zone.radius_m / config.resolution_m))
        for row in range(max(0, center[1] - radius_cells), min(cell_count, center[1] + radius_cells + 1)):
            for col in range(max(0, center[0] - radius_cells), min(cell_count, center[0] + radius_cells + 1)):
                if (row - center[1]) ** 2 + (col - center[0]) ** 2 <= radius_cells**2:
                    grid[row, col] = max(grid[row, col], int(zone.cost))


__all__ = [
    "CommandProposal",
    "LocalCostmapConfig",
    "SocialZone",
    "TrajectoryCandidate",
    "TrajectoryConfig",
    "choose_command",
    "make_local_costmap",
    "rollout_arc",
    "sample_trajectories",
]
