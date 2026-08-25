"""Pure point-cell support filtering for obstacle clouds."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from wheelchair_navigation.costmap import FrontCostmapConfig, front_point_cell_ids


@dataclass(frozen=True)
class PointSupportStats:
    """Counts produced by one support-filter pass."""

    min_points_per_cell: int
    occupied_cells: int
    low_support_cells: int
    low_support_points: int


@dataclass(frozen=True)
class PointSupportResult:
    """Full-cloud masks and counters produced by support filtering."""

    keep_mask: np.ndarray
    low_support_mask: np.ndarray
    low_support_cell_ids: np.ndarray
    stats: PointSupportStats


def filter_points_by_cell_support(
    points_base: np.ndarray,
    eligible_mask: np.ndarray,
    config: FrontCostmapConfig,
    *,
    min_points_per_cell: int,
) -> PointSupportResult:
    """Remove eligible points whose 2-D front-grid cell lacks support.

    Points outside Nav2's marking window are retained when finite so Nav2
    remains responsible for its own range, height, and ray-tracing policy.
    """

    points = _points_array(points_base)
    eligible = np.asarray(eligible_mask, dtype=bool)
    if eligible.shape != (points.shape[0],):
        raise ValueError("eligible_mask must have shape (N,)")
    minimum = _validated_minimum_points(min_points_per_cell)

    eligible_indices = np.flatnonzero(eligible)
    valid, cell_ids, cell_count = front_point_cell_ids(
        points[eligible_indices], config
    )
    counts = np.bincount(cell_ids[valid], minlength=cell_count)
    occupied = np.flatnonzero(counts > 0)
    low_support_cells = occupied[counts[occupied] < minimum]
    eligible_low_support = valid & np.isin(cell_ids, low_support_cells)

    low_support = np.zeros(points.shape[0], dtype=bool)
    low_support[eligible_indices] = eligible_low_support
    finite = np.isfinite(points).all(axis=1)
    return PointSupportResult(
        keep_mask=finite & ~low_support,
        low_support_mask=low_support,
        low_support_cell_ids=low_support_cells,
        stats=PointSupportStats(
            min_points_per_cell=minimum,
            occupied_cells=int(occupied.size),
            low_support_cells=int(low_support_cells.size),
            low_support_points=int(np.count_nonzero(low_support)),
        ),
    )


def _points_array(points: np.ndarray) -> np.ndarray:
    array = np.asarray(points, dtype=np.float32)
    if array.size == 0:
        array = np.empty((0, 3), dtype=np.float32)
    if array.ndim != 2 or array.shape[1] != 3:
        raise ValueError("points_base must have shape (N, 3)")
    return array


def _validated_minimum_points(value: int) -> int:
    if isinstance(value, (bool, np.bool_)):
        raise ValueError("min_points_per_cell must be an integer")
    try:
        minimum = int(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError("min_points_per_cell must be an integer") from exc
    if minimum != value or minimum < 1:
        raise ValueError("min_points_per_cell must be at least one")
    return minimum


__all__ = [
    "PointSupportResult",
    "PointSupportStats",
    "filter_points_by_cell_support",
]
