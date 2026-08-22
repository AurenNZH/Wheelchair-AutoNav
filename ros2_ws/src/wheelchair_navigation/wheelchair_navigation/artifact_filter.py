"""Base-frame cell geometry for the diagnostic AIRY artifact shadow map."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from wheelchair_navigation.costmap import (
    FrontCostmapConfig,
    SelfFilterBox,
    front_point_cell_ids,
    points_in_box,
)


@dataclass(frozen=True)
class ArtifactGridCell:
    """One height-bounded front-costmap cell assigned to a visual region."""

    region_id: int
    forward_cell: int
    lateral_cell: int
    min_z_m: float
    max_z_m: float


@dataclass(frozen=True)
class ArtifactHaloSpan:
    """One inclusive lateral run in a configured region halo."""

    region_id: int
    forward_cell: int
    min_lateral_cell: int
    max_lateral_cell: int


@dataclass(frozen=True)
class ArtifactFilterStats:
    """Counts produced while applying possibly-overlapping regions."""

    region_count: int
    per_region_rejected_points: tuple[int, ...]
    unique_rejected_points: int


@dataclass(frozen=True)
class ArtifactCellSupportStats:
    """Point-support counters for mask and halo cells."""

    min_points_per_cell: int
    global_min_points_per_cell: int
    configured_halo_cells: int
    mask_touched_cells: int
    mask_removed_cells: int
    mask_mixed_cells: int
    threshold_candidate_cells: int
    low_support_cells: int
    low_support_points: int


@dataclass(frozen=True)
class ArtifactCellSupportResult:
    """Full-cloud masks and counters after cell-support filtering."""

    shadow_mask: np.ndarray
    low_support_mask: np.ndarray
    candidate_cell_ids: np.ndarray
    low_support_cell_ids: np.ndarray
    stats: ArtifactCellSupportStats


@dataclass(frozen=True)
class ArtifactPointFilterResult:
    """Masks and statistics shared by grid and PointCloud filter outputs."""

    keep_mask: np.ndarray
    front_valid_mask: np.ndarray
    self_rejected_mask: np.ndarray
    artifact_rejected_mask: np.ndarray
    support: ArtifactCellSupportResult
    artifact_stats: ArtifactFilterStats


def filter_artifact_points(
    points_base: np.ndarray,
    eligible_mask: np.ndarray,
    self_filter_boxes: tuple[SelfFilterBox, ...],
    cells: tuple[ArtifactGridCell, ...],
    halo_spans: tuple[ArtifactHaloSpan, ...],
    config: FrontCostmapConfig,
    *,
    min_points_per_cell: int,
    global_min_points_per_cell: int,
    threshold_candidate_cell_ids: np.ndarray | None = None,
) -> ArtifactPointFilterResult:
    """Apply the calibrated masks and support thresholds to one cloud.

    ``eligible_mask`` defines points that Nav2 may mark in the calibrated front
    grid. The returned upstream keep mask deliberately retains finite points
    outside that marking window so Nav2 remains responsible for its own height,
    range, and ray-tracing policy.
    """

    points = _points_array(points_base, "points_base")
    eligible = np.asarray(eligible_mask, dtype=bool)
    if eligible.shape != (points.shape[0],):
        raise ValueError("artifact eligible mask must have shape (N,)")

    finite = np.isfinite(points).all(axis=1)
    finite_indices = np.flatnonzero(finite)
    self_rejected = np.zeros(points.shape[0], dtype=bool)
    for box in self_filter_boxes:
        self_rejected[finite_indices] |= points_in_box(
            points[finite_indices], box
        )

    eligible_indices = np.flatnonzero(eligible)
    eligible_points = points[eligible_indices]
    eligible_front_valid, cell_ids, cell_count = front_point_cell_ids(
        eligible_points, config
    )
    eligible_artifact_rejected, artifact_stats = artifact_grid_membership(
        eligible_points,
        cell_ids,
        eligible_front_valid,
        cells,
        config,
    )
    threshold_cells = (
        artifact_configured_halo_cell_ids(halo_spans, cells, config)
        if threshold_candidate_cell_ids is None
        else np.asarray(threshold_candidate_cell_ids, dtype=np.int64)
    )
    eligible_support = minimum_cell_support_filter(
        cell_ids,
        eligible_front_valid,
        np.ones(eligible_indices.size, dtype=bool),
        eligible_artifact_rejected,
        threshold_cells,
        cell_count=cell_count,
        min_points_per_cell=min_points_per_cell,
        global_min_points_per_cell=global_min_points_per_cell,
    )

    front_valid = np.zeros(points.shape[0], dtype=bool)
    artifact_rejected = np.zeros(points.shape[0], dtype=bool)
    low_support = np.zeros(points.shape[0], dtype=bool)
    shadow = np.zeros(points.shape[0], dtype=bool)
    front_valid[eligible_indices] = eligible_front_valid
    artifact_rejected[eligible_indices] = eligible_artifact_rejected
    low_support[eligible_indices] = eligible_support.low_support_mask
    shadow[eligible_indices] = eligible_support.shadow_mask
    support = ArtifactCellSupportResult(
        shadow_mask=shadow,
        low_support_mask=low_support,
        candidate_cell_ids=eligible_support.candidate_cell_ids,
        low_support_cell_ids=eligible_support.low_support_cell_ids,
        stats=eligible_support.stats,
    )
    keep = (
        finite
        & ~self_rejected
        & ~artifact_rejected
        & ~low_support
    )
    return ArtifactPointFilterResult(
        keep_mask=keep,
        front_valid_mask=front_valid,
        self_rejected_mask=self_rejected,
        artifact_rejected_mask=artifact_rejected,
        support=support,
        artifact_stats=artifact_stats,
    )


def parse_artifact_grid_cells(
    values: list[float] | tuple[float, ...] | None,
) -> tuple[ArtifactGridCell, ...]:
    """Parse flat groups of five base-frame grid-cell values."""

    if values is None:
        return ()
    if len(values) % 5:
        raise ValueError(
            "artifact_grid_mask_cells must contain complete groups of five values"
        )

    cells = []
    seen = set()
    for start in range(0, len(values), 5):
        fields = tuple(float(value) for value in values[start:start + 5])
        if not np.isfinite(fields).all():
            raise ValueError("artifact grid mask values must be finite")
        integer_fields = []
        for value in fields[:3]:
            integer = int(value)
            if float(integer) != value:
                raise ValueError(
                    "artifact region and cell coordinates must be integers"
                )
            integer_fields.append(integer)
        region_id, forward_cell, lateral_cell = integer_fields
        if region_id < 0:
            raise ValueError("artifact region ID must be non-negative")
        if forward_cell < 0:
            raise ValueError("artifact forward cell must be non-negative")
        min_z_m, max_z_m = fields[3:]
        if min_z_m > max_z_m:
            raise ValueError("artifact grid mask minimum Z exceeds maximum Z")
        key = (region_id, forward_cell, lateral_cell)
        if key in seen:
            raise ValueError("duplicate artifact cell within one region")
        seen.add(key)
        cells.append(
            ArtifactGridCell(
                region_id,
                forward_cell,
                lateral_cell,
                min_z_m,
                max_z_m,
            )
        )
    return tuple(cells)


def parse_artifact_grid_halo_spans(
    values: list[float] | tuple[float, ...] | None,
) -> tuple[ArtifactHaloSpan, ...]:
    """Parse flat groups of four explicit base-frame halo span values."""

    if values is None:
        return ()
    if len(values) % 4:
        raise ValueError(
            "artifact_grid_halo_spans must contain complete groups of four values"
        )

    spans = []
    seen_cells = set()
    for start in range(0, len(values), 4):
        fields = tuple(float(value) for value in values[start:start + 4])
        if not np.isfinite(fields).all():
            raise ValueError("artifact grid halo values must be finite")
        integer_fields = []
        for value in fields:
            integer = int(value)
            if float(integer) != value:
                raise ValueError("artifact halo coordinates must be integers")
            integer_fields.append(integer)
        region_id, forward_cell, min_lateral, max_lateral = integer_fields
        if region_id < 0:
            raise ValueError("artifact halo region ID must be non-negative")
        if forward_cell < 0:
            raise ValueError("artifact halo forward cell must be non-negative")
        if min_lateral > max_lateral:
            raise ValueError("artifact halo lateral bounds are reversed")
        for lateral_cell in range(min_lateral, max_lateral + 1):
            key = (region_id, forward_cell, lateral_cell)
            if key in seen_cells:
                raise ValueError("overlapping artifact halo spans within one region")
            seen_cells.add(key)
        spans.append(
            ArtifactHaloSpan(
                region_id,
                forward_cell,
                min_lateral,
                max_lateral,
            )
        )
    return tuple(spans)


def validate_artifact_filter_frame(
    artifact_filter_frame: str,
    target_frame: str,
) -> None:
    """Require configured cell geometry to use the transformed target frame."""

    if not artifact_filter_frame:
        raise ValueError("artifact_filter_frame must not be empty")
    if artifact_filter_frame != target_frame:
        raise ValueError(
            "artifact filter frame mismatch: geometry is '%s', target is '%s'"
            % (artifact_filter_frame, target_frame or "<empty>")
        )


def validate_artifact_grid_cells(
    cells: tuple[ArtifactGridCell, ...],
    config: FrontCostmapConfig,
) -> None:
    """Validate region IDs and cell coordinates against the front grid."""

    width, height, zero_row = _front_grid_geometry(config)
    region_ids = sorted({cell.region_id for cell in cells})
    if region_ids and region_ids != list(range(region_ids[-1] + 1)):
        raise ValueError("artifact region IDs must be contiguous from zero")
    for cell in cells:
        row = zero_row + cell.lateral_cell
        if cell.forward_cell >= width or row < 0 or row >= height:
            raise ValueError("artifact grid mask cell is outside the front grid")


def validate_artifact_grid_halo_spans(
    spans: tuple[ArtifactHaloSpan, ...],
    cells: tuple[ArtifactGridCell, ...],
    config: FrontCostmapConfig,
) -> None:
    """Require explicit halo spans to cover their regions and mask cells."""

    validate_artifact_grid_cells(cells, config)
    width, height, zero_row = _front_grid_geometry(config)
    mask_regions = {cell.region_id for cell in cells}
    halo_cells = set()
    for span in spans:
        if span.region_id not in mask_regions:
            raise ValueError("artifact halo span refers to an unknown region")
        min_row = zero_row + span.min_lateral_cell
        max_row = zero_row + span.max_lateral_cell
        if (
            span.forward_cell >= width
            or min_row < 0
            or max_row >= height
        ):
            raise ValueError("artifact halo span is outside the front grid")
        for lateral_cell in range(
            span.min_lateral_cell, span.max_lateral_cell + 1
        ):
            halo_cells.add(
                (span.region_id, span.forward_cell, lateral_cell)
            )
    mask_cells = {
        (cell.region_id, cell.forward_cell, cell.lateral_cell)
        for cell in cells
    }
    if not mask_cells.issubset(halo_cells):
        raise ValueError("artifact halo spans must contain every mask cell")


def artifact_grid_membership(
    points_base: np.ndarray,
    cell_ids: np.ndarray,
    valid_front_mask: np.ndarray,
    cells: tuple[ArtifactGridCell, ...],
    config: FrontCostmapConfig,
    eligible_mask: np.ndarray | None = None,
) -> tuple[np.ndarray, ArtifactFilterStats]:
    """Return height-bounded cell-union membership and region counts."""

    points = _points_array(points_base, "points_base")
    ids = np.asarray(cell_ids, dtype=np.int64)
    valid = np.asarray(valid_front_mask, dtype=bool)
    eligible = (
        np.ones(points.shape[0], dtype=bool)
        if eligible_mask is None
        else np.asarray(eligible_mask, dtype=bool)
    )
    expected_shape = (points.shape[0],)
    if any(value.shape != expected_shape for value in (ids, valid, eligible)):
        raise ValueError("artifact point arrays must have matching shape (N,)")

    validate_artifact_grid_cells(cells, config)
    width, height, zero_row = _front_grid_geometry(config)
    cell_count = width * height
    if np.any(valid & ((ids < 0) | (ids >= cell_count))):
        raise ValueError("valid artifact point cell ID is outside the grid")

    finite = np.isfinite(points).all(axis=1)
    safe_ids = np.where(valid, ids, 0)
    union = np.zeros(points.shape[0], dtype=bool)
    per_region = []
    region_ids = sorted({cell.region_id for cell in cells})
    for region_id in region_ids:
        min_z = np.full(cell_count, np.inf, dtype=np.float32)
        max_z = np.full(cell_count, -np.inf, dtype=np.float32)
        for cell in cells:
            if cell.region_id != region_id:
                continue
            cell_id = (
                (zero_row + cell.lateral_cell) * width
                + cell.forward_cell
            )
            min_z[cell_id] = cell.min_z_m
            max_z[cell_id] = cell.max_z_m
        with np.errstate(invalid="ignore"):
            inside = (
                finite
                & valid
                & eligible
                & (points[:, 2] >= min_z[safe_ids])
                & (points[:, 2] <= max_z[safe_ids])
            )
        per_region.append(int(np.count_nonzero(inside)))
        union |= inside

    return union, ArtifactFilterStats(
        region_count=len(region_ids),
        per_region_rejected_points=tuple(per_region),
        unique_rejected_points=int(np.count_nonzero(union)),
    )


def artifact_region_cell_ids(
    cells: tuple[ArtifactGridCell, ...],
    config: FrontCostmapConfig,
    region_id: int | None = None,
) -> np.ndarray:
    """Return sorted unique front-grid IDs for all or one configured region."""

    validate_artifact_grid_cells(cells, config)
    width, _, zero_row = _front_grid_geometry(config)
    selected = [
        (zero_row + cell.lateral_cell) * width + cell.forward_cell
        for cell in cells
        if region_id is None or cell.region_id == region_id
    ]
    return np.unique(np.asarray(selected, dtype=np.int64))


def artifact_configured_halo_cell_ids(
    spans: tuple[ArtifactHaloSpan, ...],
    cells: tuple[ArtifactGridCell, ...],
    config: FrontCostmapConfig,
    region_id: int | None = None,
) -> np.ndarray:
    """Return sorted unique cell IDs from the configured halo shape."""

    validate_artifact_grid_halo_spans(spans, cells, config)
    width, _, zero_row = _front_grid_geometry(config)
    selected = []
    for span in spans:
        if region_id is not None and span.region_id != region_id:
            continue
        selected.extend(
            (zero_row + lateral_cell) * width + span.forward_cell
            for lateral_cell in range(
                span.min_lateral_cell, span.max_lateral_cell + 1
            )
        )
    return np.unique(np.asarray(selected, dtype=np.int64))


def artifact_halo_cell_ids(
    cells: tuple[ArtifactGridCell, ...],
    config: FrontCostmapConfig,
    halo_cells: int,
    region_id: int | None = None,
) -> np.ndarray:
    """Dilate a staircase footprint by an eight-neighbour cell radius."""

    radius = _validated_halo_cells(halo_cells)
    validate_artifact_grid_cells(cells, config)
    width, height, zero_row = _front_grid_geometry(config)
    selected = {
        (cell.forward_cell, zero_row + cell.lateral_cell)
        for cell in cells
        if region_id is None or cell.region_id == region_id
    }
    dilated = set()
    for col, row in selected:
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                next_col = col + dx
                next_row = row + dy
                if 0 <= next_col < width and 0 <= next_row < height:
                    dilated.add(next_row * width + next_col)
    return np.asarray(sorted(dilated), dtype=np.int64)


def minimum_cell_support_filter(
    cell_ids: np.ndarray,
    valid_front_mask: np.ndarray,
    eligible_mask: np.ndarray,
    mask_rejected_mask: np.ndarray,
    threshold_candidate_cell_ids: np.ndarray,
    *,
    cell_count: int,
    min_points_per_cell: int,
    global_min_points_per_cell: int = 1,
) -> ArtifactCellSupportResult:
    """Require global support plus a stricter mask-and-halo threshold."""

    minimum = _validated_minimum_points(min_points_per_cell)
    global_minimum = _validated_minimum_points(
        global_min_points_per_cell
    )
    if cell_count < 1:
        raise ValueError("artifact support grid must contain at least one cell")

    ids = np.asarray(cell_ids, dtype=np.int64)
    valid = np.asarray(valid_front_mask, dtype=bool)
    eligible = np.asarray(eligible_mask, dtype=bool)
    rejected = np.asarray(mask_rejected_mask, dtype=bool)
    candidates = np.asarray(threshold_candidate_cell_ids, dtype=np.int64)
    expected_shape = ids.shape
    if ids.ndim != 1 or any(
        value.shape != expected_shape for value in (valid, eligible, rejected)
    ):
        raise ValueError("artifact support masks must have matching shape (N,)")
    if candidates.ndim != 1:
        raise ValueError("artifact candidate cell IDs must have shape (N,)")
    if np.any(valid & ((ids < 0) | (ids >= cell_count))):
        raise ValueError("valid artifact support cell ID is outside the grid")
    if np.any((candidates < 0) | (candidates >= cell_count)):
        raise ValueError("artifact candidate cell ID is outside the grid")

    kept_after_mask = eligible & ~rejected
    kept_front = kept_after_mask & valid
    kept_counts = np.bincount(ids[kept_front], minlength=cell_count)
    mask_cells = np.unique(ids[eligible & rejected & valid])
    candidate_cells = np.union1d(mask_cells, candidates)
    candidate_counts = kept_counts[candidate_cells]
    local_low_support_cells = candidate_cells[
        (candidate_counts > 0) & (candidate_counts < minimum)
    ]
    occupied_cells = np.flatnonzero(kept_counts > 0)
    global_low_support_cells = occupied_cells[
        kept_counts[occupied_cells] < global_minimum
    ]
    low_support_cells = np.union1d(
        local_low_support_cells, global_low_support_cells
    )
    low_support = kept_front & np.isin(ids, low_support_cells)
    shadow = kept_after_mask & ~low_support

    mask_counts = kept_counts[mask_cells]
    return ArtifactCellSupportResult(
        shadow_mask=shadow,
        low_support_mask=low_support,
        candidate_cell_ids=candidate_cells,
        low_support_cell_ids=low_support_cells,
        stats=ArtifactCellSupportStats(
            min_points_per_cell=minimum,
            global_min_points_per_cell=global_minimum,
            configured_halo_cells=int(candidate_cells.size),
            mask_touched_cells=int(mask_cells.size),
            mask_removed_cells=int(np.count_nonzero(mask_counts == 0)),
            mask_mixed_cells=int(np.count_nonzero(mask_counts > 0)),
            threshold_candidate_cells=int(candidate_cells.size),
            low_support_cells=int(low_support_cells.size),
            low_support_points=int(np.count_nonzero(low_support)),
        ),
    )


def _front_grid_geometry(
    config: FrontCostmapConfig,
) -> tuple[int, int, int]:
    geometry = np.asarray(
        [config.length_m, config.width_m, config.resolution_m],
        dtype=np.float64,
    )
    if not np.isfinite(geometry).all() or np.any(geometry <= 0.0):
        raise ValueError("artifact front-grid geometry must be positive")
    width = int(np.ceil(config.length_m / config.resolution_m))
    height = int(np.ceil(config.width_m / config.resolution_m))
    zero_row_value = height / 2.0
    if not float(zero_row_value).is_integer():
        raise ValueError("artifact grid requires a cell boundary at lateral zero")
    return width, height, int(zero_row_value)


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


def _validated_halo_cells(value: int) -> int:
    if isinstance(value, (bool, np.bool_)):
        raise ValueError("artifact halo cells must be an integer")
    try:
        radius = int(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError("artifact halo cells must be an integer") from exc
    if radius != value or radius < 0:
        raise ValueError("artifact halo cells must be non-negative")
    return radius


__all__ = [
    "ArtifactCellSupportResult",
    "ArtifactCellSupportStats",
    "ArtifactFilterStats",
    "ArtifactGridCell",
    "ArtifactHaloSpan",
    "ArtifactPointFilterResult",
    "artifact_configured_halo_cell_ids",
    "filter_artifact_points",
    "artifact_grid_membership",
    "artifact_halo_cell_ids",
    "artifact_region_cell_ids",
    "minimum_cell_support_filter",
    "parse_artifact_grid_cells",
    "parse_artifact_grid_halo_spans",
    "validate_artifact_filter_frame",
    "validate_artifact_grid_cells",
    "validate_artifact_grid_halo_spans",
]
