"""Sensor-frame geometry for the diagnostic AIRY artifact shadow map."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class ArtifactPancakeMask:
    """A flat oriented rectangular prism expressed in the AIRY frame."""

    start_x_m: float
    start_y_m: float
    end_x_m: float
    end_y_m: float
    half_width_m: float
    min_z_m: float
    max_z_m: float

    @property
    def length_m(self) -> float:
        return float(
            np.hypot(
                self.end_x_m - self.start_x_m,
                self.end_y_m - self.start_y_m,
            )
        )


@dataclass(frozen=True)
class ArtifactFilterStats:
    """Counts produced while applying all possibly-overlapping masks."""

    mask_count: int
    per_mask_rejected_points: tuple[int, ...]
    unique_rejected_points: int


@dataclass(frozen=True)
class ArtifactCellSupportStats:
    """Point-support counters for cells near configured artifact masks."""

    min_points_per_cell: int
    halo_m: float
    prism_touched_cells: int
    prism_removed_cells: int
    prism_mixed_cells: int
    threshold_candidate_cells: int
    low_support_cells: int
    low_support_points: int


@dataclass(frozen=True)
class ArtifactCellSupportResult:
    """Full-cloud masks and counters after cell-support filtering."""

    shadow_mask: np.ndarray
    low_support_mask: np.ndarray
    stats: ArtifactCellSupportStats


def parse_artifact_pancake_masks(
    values: list[float] | tuple[float, ...] | None,
) -> tuple[ArtifactPancakeMask, ...]:
    """Parse flat groups of seven sensor-frame prism values."""

    if values is None:
        return ()
    if len(values) % 7:
        raise ValueError(
            "artifact_pancake_masks must contain complete groups of seven values"
        )

    masks = []
    for start in range(0, len(values), 7):
        fields = tuple(float(value) for value in values[start:start + 7])
        if not np.isfinite(fields).all():
            raise ValueError("artifact pancake mask values must be finite")
        mask = ArtifactPancakeMask(*fields)
        if mask.length_m <= 0.0:
            raise ValueError("artifact pancake mask segment must have non-zero length")
        if mask.half_width_m <= 0.0:
            raise ValueError("artifact pancake mask half-width must be positive")
        if mask.min_z_m > mask.max_z_m:
            raise ValueError("artifact pancake mask minimum Z exceeds maximum Z")
        masks.append(mask)
    return tuple(masks)


def validate_artifact_filter_frame(
    source_frame: str,
    artifact_filter_frame: str,
) -> None:
    """Require native cloud coordinates to match the configured mask frame."""

    if not artifact_filter_frame:
        raise ValueError("artifact_filter_frame must not be empty")
    if source_frame != artifact_filter_frame:
        raise ValueError(
            "artifact filter frame mismatch: cloud is '%s', masks are '%s'"
            % (source_frame or "<empty>", artifact_filter_frame)
        )


def artifact_pancake_membership(
    points_sensor: np.ndarray,
    masks: tuple[ArtifactPancakeMask, ...],
    eligible_mask: np.ndarray | None = None,
) -> tuple[np.ndarray, ArtifactFilterStats]:
    """Return the union mask and counts for flat oriented prisms.

    Projection is performed directly in the sensor XY plane. Counts for each
    mask may overlap, while ``unique_rejected_points`` counts their union.
    """

    points = _points_array(points_sensor, "points_sensor")
    eligible = (
        np.ones(points.shape[0], dtype=bool)
        if eligible_mask is None
        else np.asarray(eligible_mask, dtype=bool)
    )
    if eligible.shape != (points.shape[0],):
        raise ValueError("eligible_mask must have shape (N,)")

    finite = np.isfinite(points).all(axis=1)
    x = points[:, 0]
    y = points[:, 1]
    z = points[:, 2]
    union = np.zeros(points.shape[0], dtype=bool)
    per_mask = []
    for mask in masks:
        dx = mask.end_x_m - mask.start_x_m
        dy = mask.end_y_m - mask.start_y_m
        length_squared = dx * dx + dy * dy
        with np.errstate(invalid="ignore"):
            candidate = (
                finite
                & eligible
                & (
                    x
                    >= min(mask.start_x_m, mask.end_x_m)
                    - mask.half_width_m
                )
                & (
                    x
                    <= max(mask.start_x_m, mask.end_x_m)
                    + mask.half_width_m
                )
                & (
                    y
                    >= min(mask.start_y_m, mask.end_y_m)
                    - mask.half_width_m
                )
                & (
                    y
                    <= max(mask.start_y_m, mask.end_y_m)
                    + mask.half_width_m
                )
                & (z >= mask.min_z_m)
                & (z <= mask.max_z_m)
            )
        candidate_indices = np.flatnonzero(candidate)
        candidate_points = points[candidate_indices]
        along = (
            (candidate_points[:, 0] - mask.start_x_m) * dx
            + (candidate_points[:, 1] - mask.start_y_m) * dy
        ) / length_squared
        perpendicular_m = np.abs(
            (candidate_points[:, 0] - mask.start_x_m) * dy
            - (candidate_points[:, 1] - mask.start_y_m) * dx
        ) / np.sqrt(length_squared)
        with np.errstate(invalid="ignore"):
            inside_candidate = (
                (along >= 0.0)
                & (along <= 1.0)
                & (perpendicular_m <= mask.half_width_m)
            )
        inside_indices = candidate_indices[inside_candidate]
        per_mask.append(int(inside_indices.size))
        union[inside_indices] = True

    return union, ArtifactFilterStats(
        mask_count=len(masks),
        per_mask_rejected_points=tuple(per_mask),
        unique_rejected_points=int(np.count_nonzero(union)),
    )


def artifact_xy_halo_membership(
    points_sensor: np.ndarray,
    masks: tuple[ArtifactPancakeMask, ...],
    halo_m: float,
    eligible_mask: np.ndarray | None = None,
) -> np.ndarray:
    """Return points near a mask's XY footprint, ignoring point height."""

    if not np.isfinite(halo_m) or halo_m < 0.0:
        raise ValueError("artifact threshold halo must be finite and non-negative")
    points = _points_array(points_sensor, "points_sensor")
    eligible = (
        np.ones(points.shape[0], dtype=bool)
        if eligible_mask is None
        else np.asarray(eligible_mask, dtype=bool)
    )
    if eligible.shape != (points.shape[0],):
        raise ValueError("eligible_mask must have shape (N,)")

    finite = np.isfinite(points[:, :2]).all(axis=1)
    union = np.zeros(points.shape[0], dtype=bool)
    for mask in masks:
        dx = mask.end_x_m - mask.start_x_m
        dy = mask.end_y_m - mask.start_y_m
        length_m = mask.length_m
        along_m = (
            (points[:, 0] - mask.start_x_m) * dx
            + (points[:, 1] - mask.start_y_m) * dy
        ) / length_m
        perpendicular_m = np.abs(
            (points[:, 0] - mask.start_x_m) * dy
            - (points[:, 1] - mask.start_y_m) * dx
        ) / length_m
        with np.errstate(invalid="ignore"):
            union |= (
                finite
                & eligible
                & (along_m >= -halo_m)
                & (along_m <= length_m + halo_m)
                & (perpendicular_m <= mask.half_width_m + halo_m)
            )
    return union


def minimum_cell_support_filter(
    cell_ids: np.ndarray,
    valid_front_mask: np.ndarray,
    eligible_mask: np.ndarray,
    prism_rejected_mask: np.ndarray,
    halo_mask: np.ndarray,
    *,
    cell_count: int,
    min_points_per_cell: int,
    halo_m: float,
) -> ArtifactCellSupportResult:
    """Require point support only in cells touched by or near a prism."""

    minimum = _validated_minimum_points(min_points_per_cell)
    if not np.isfinite(halo_m) or halo_m < 0.0:
        raise ValueError("artifact threshold halo must be finite and non-negative")
    if cell_count < 1:
        raise ValueError("artifact support grid must contain at least one cell")

    ids = np.asarray(cell_ids, dtype=np.int64)
    valid = np.asarray(valid_front_mask, dtype=bool)
    eligible = np.asarray(eligible_mask, dtype=bool)
    prism_rejected = np.asarray(prism_rejected_mask, dtype=bool)
    halo = np.asarray(halo_mask, dtype=bool)
    expected_shape = ids.shape
    if ids.ndim != 1 or any(
        value.shape != expected_shape
        for value in (valid, eligible, prism_rejected, halo)
    ):
        raise ValueError("artifact support masks must have matching shape (N,)")
    if np.any(valid & ((ids < 0) | (ids >= cell_count))):
        raise ValueError("valid artifact support cell ID is outside the grid")

    kept_after_prism = eligible & ~prism_rejected
    kept_front = kept_after_prism & valid
    kept_counts = np.bincount(ids[kept_front], minlength=cell_count)

    prism_cells = np.unique(ids[eligible & prism_rejected & valid])
    halo_cells = np.unique(ids[kept_front & halo])
    candidate_cells = np.union1d(prism_cells, halo_cells)
    candidate_counts = kept_counts[candidate_cells]
    low_support_cells = candidate_cells[
        (candidate_counts > 0) & (candidate_counts < minimum)
    ]
    low_support = kept_front & np.isin(ids, low_support_cells)
    shadow = kept_after_prism & ~low_support

    prism_counts = kept_counts[prism_cells]
    prism_removed_cells = int(np.count_nonzero(prism_counts == 0))
    prism_mixed_cells = int(np.count_nonzero(prism_counts > 0))
    return ArtifactCellSupportResult(
        shadow_mask=shadow,
        low_support_mask=low_support,
        stats=ArtifactCellSupportStats(
            min_points_per_cell=minimum,
            halo_m=float(halo_m),
            prism_touched_cells=int(prism_cells.size),
            prism_removed_cells=prism_removed_cells,
            prism_mixed_cells=prism_mixed_cells,
            threshold_candidate_cells=int(candidate_cells.size),
            low_support_cells=int(low_support_cells.size),
            low_support_points=int(np.count_nonzero(low_support)),
        ),
    )


def make_artifact_shadow_points(
    points_sensor: np.ndarray,
    points_base: np.ndarray,
    eligible_mask: np.ndarray,
    masks: tuple[ArtifactPancakeMask, ...],
    *,
    source_frame: str,
    artifact_filter_frame: str,
) -> tuple[np.ndarray, np.ndarray, ArtifactFilterStats]:
    """Return eligible base points minus masks and rejected sensor points."""

    sensor = _points_array(points_sensor, "points_sensor")
    base = _points_array(points_base, "points_base")
    if sensor.shape != base.shape:
        raise ValueError("sensor and base point arrays must have matching shapes")
    eligible = np.asarray(eligible_mask, dtype=bool)
    if eligible.shape != (sensor.shape[0],):
        raise ValueError("eligible_mask must have shape (N,)")

    validate_artifact_filter_frame(source_frame, artifact_filter_frame)
    rejected_mask, stats = artifact_pancake_membership(sensor, masks, eligible)
    return base[eligible & ~rejected_mask], sensor[rejected_mask], stats


def _points_array(points: np.ndarray, name: str) -> np.ndarray:
    array = np.asarray(points, dtype=np.float32)
    if array.size == 0:
        array = np.empty((0, 3), dtype=np.float32)
    if array.ndim != 2 or array.shape[1] != 3:
        raise ValueError("%s must have shape (N, 3)" % name)
    return array


def _validated_minimum_points(value: int) -> int:
    if isinstance(value, (bool, np.bool_)):
        raise ValueError("artifact minimum points per cell must be an integer")
    try:
        minimum = int(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(
            "artifact minimum points per cell must be an integer"
        ) from exc
    if minimum != value or minimum < 1:
        raise ValueError("artifact minimum points per cell must be at least one")
    return minimum


__all__ = [
    "ArtifactCellSupportResult",
    "ArtifactCellSupportStats",
    "ArtifactFilterStats",
    "ArtifactPancakeMask",
    "artifact_pancake_membership",
    "artifact_xy_halo_membership",
    "make_artifact_shadow_points",
    "minimum_cell_support_filter",
    "parse_artifact_pancake_masks",
    "validate_artifact_filter_frame",
]
